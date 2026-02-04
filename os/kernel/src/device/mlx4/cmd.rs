//! This module consists of functions to create a direct memory access mailbox for passing parameters to the hca
//! and getting output back from the hca during verb calls and functions to execute verb calls.

use core::sync::atomic::{compiler_fence, Ordering};

use super::utils::{OperationArgs, Operations};
use crate::device::mlx4::utils;
use crate::{
    device::mlx4::utils::{CopyOperation, FillOperation},
    memory::PAGE_SIZE,
};
use alloc::boxed::Box;
use bitflags::bitflags;
use log::trace;
use strum_macros::{FromRepr, IntoStaticStr};
use tock_registers::interfaces::{Readable, Writeable};
use tock_registers::register_bitfields;
use tock_registers::registers::{ReadWrite, WriteOnly};

const HCR_BASE: usize = 0x80680;
const HCR_OPMOD_SHIFT: u32 = 12;
const HCR_T_BIT: u32 = 21;
const HCR_E_BIT: u32 = 22;
const HCR_GO_BIT: u32 = 23;
const POLL_TOKEN: u32 = 0xffff;

#[repr(u16)]
#[derive(Debug)]
#[allow(dead_code)]
pub(super) enum Opcode {
    // initialization and general commands
    QueryDevCap = 0x03,
    QueryFw = 0x04,
    QueryAdapter = 0x06,
    InitHca = 0x07,
    CloseHca = 0x08,
    InitPort = 0x09,
    ClosePort = 0x0a,
    QueryHca = 0x0b,
    QueryPort = 0x43,
    SetPort = 0x0c,
    RunFw = 0xff6,
    UnmapIcm = 0xff9,
    MapIcm = 0xffa,
    UnmapIcmAux = 0xffb,
    MapIcmAux = 0xffc,
    UnmapFa = 0xffe,
    SetIcmSize = 0xffd,
    MapFa = 0xfff,

    // TPT commands
    Sw2HwMpt = 0x0d,
    QueryMpt = 0x0e,
    Hw2SwMpt = 0x0f,
    ReadMtt = 0x10,
    WriteMtt = 0x11,

    // EQ commands
    MapEq = 0x12,
    Sw2HwEq = 0x13,
    Hw2SwEq = 0x14,
    QueryEq = 0x15,
    GenEqe = 0x58,

    // CQ commands
    Sw2HwCq = 0x16,
    Hw2SwCq = 0x17,
    QueryCq = 0x18,
    ModifyCq = 0x2c,

    // QP/EE commands
    // The "Any" is there because identifiers cannot start with a number.
    Rst2InitQp = 0x19,
    Init2RtrQp = 0x1a,
    Rtr2RtsQp = 0x1b,
    Rts2RtsQp = 0x1c,
    Sqerr2RtsQp = 0x1d,
    Any2ErrQp = 0x1e,
    Rts2SqdQp = 0x1f,
    Sqd2RtsQp = 0x20,
    Any2RstQp = 0x21,
    QueryQp = 0x22,
    Init2InitQp = 0x2d,
    SuspendQp = 0x32,
    UnsuspendQp = 0x33,
    Sqd2SqdQp = 0x38,
    UpdateQp = 0x61,
    State2StateQp = 0x82,

    // special QP and management commands
    ConfSpecialQp = 0x23,
    MadIfc = 0x24,
    MadDemux = 0x203,
    // miscellaneous commands
    // Ethernet specific commands
}

/// a modifier for the opcode
pub(super) trait OpcodeModifier {
    fn get(self) -> u8;
}

/// No modifier.
impl OpcodeModifier for () {
    fn get(self) -> u8 {
        0
    }
}

#[repr(u8)]
#[derive(Debug)]
#[allow(dead_code)]
/// Modifiers for MadDemux
pub(super) enum MadDemuxOpcodeModifier {
    Configure = 0,
    QueryState = 0x1,
    QueryRestrictions = 0x2,
}

impl OpcodeModifier for MadDemuxOpcodeModifier {
    fn get(self) -> u8 {
        self as u8
    }
}

bitflags! {
    /// Modifiers for MadIfc
    pub(super) struct MadIfcOpcodeModifier: u8 {
        const DISABLE_MKEY_VALIDATION = 1 << 0;
        const DISABLE_BKEY_VALIDATION = 1 << 1;
    }
}

impl OpcodeModifier for MadIfcOpcodeModifier {
    fn get(self) -> u8 {
        self.bits()
    }
}

#[repr(u8)]
#[derive(Debug)]
#[allow(dead_code)]
pub(super) enum SetPortOpcodeModifier {
    IB = 0x0,
    ETH = 0x1,
    BEACON = 0x4,
}

impl OpcodeModifier for SetPortOpcodeModifier {
    fn get(self) -> u8 {
        self as u8
    }
}

pub(super) struct CommandInterface<'a> {
    hcr: &'a mut Hcr,
    exp_toggle: u32,
}

register_bitfields![u32,
    StatusOpcode [
        OPCODE     OFFSET(0)  NUMBITS(12) [],
        OPCODE_MOD OFFSET(12) NUMBITS(4) [],
        T          OFFSET(21) NUMBITS(1) [],
        E          OFFSET(22) NUMBITS(1) [
            NoReport = 0,
            Report = 1
        ],
        GO         OFFSET(23) NUMBITS(1) [
            SoftwareOwnership = 0,
            HardwareOwnership = 1
    ],
        STATUS     OFFSET(24) NUMBITS(8) [],
    ]
];

#[repr(C)]
struct Hcr {
    in_param_h: WriteOnly<u32>,
    in_param_l: WriteOnly<u32>,
    in_mod: WriteOnly<u32>,
    out_param_h: ReadWrite<u32>,
    out_param_l: ReadWrite<u32>,
    /// only the first 16 bits are usable
    token: WriteOnly<u32>,
    /// status includes go, e, t and 5 reserved bits;
    /// opcode includes the opcode modifier
    status_opcode: ReadWrite<u32, StatusOpcode::Register>,
}

type MailboxAllocation = Option<utils::PageToFrameMapping>;

/// An input of a command.
///
/// This can be either `()` for commands that take no input,
/// `u64` for commands that take immediate input or
/// `&[u8]` for commands that take a mailbox.
pub(super) trait InputParameter {
    /// Possibly allocate.
    ///
    /// This will do if the input is not a mailbox.
    fn allocate(&self) -> MailboxAllocation;

    /// Get the value as u64.
    ///
    /// This is 0 for (), the value for u64 and the address of the allocated
    /// page for &[u8].
    fn as_param(&self, allocation: &MailboxAllocation) -> u64;
}

impl InputParameter for () {
    fn allocate(&self) -> MailboxAllocation {
        None
    }

    fn as_param(&self, allocation: &MailboxAllocation) -> u64 {
        assert!(allocation.is_none());
        0
    }
}
impl InputParameter for u64 {
    fn allocate(&self) -> MailboxAllocation {
        None
    }

    fn as_param(&self, allocation: &MailboxAllocation) -> u64 {
        assert!(allocation.is_none());
        *self
    }
}
impl InputParameter for &[u8] {
    fn allocate(&self) -> MailboxAllocation {
        let mut operation_container = Operations::default();

        let (mapped_pages, physical) = utils::create_cont_mapping_with_dma_flags(1).expect("").fetch_in_addr().expect("");
        let data = utils::start_page_as_mut_ptr::<u8>(mapped_pages.into_range().start);

        operation_container.add_operation(Box::new(FillOperation {}), OperationArgs::Fill(0u8, data, PAGE_SIZE));
        operation_container.add_operation(Box::new(CopyOperation {}), OperationArgs::Copy(self, data, self.len()));

        operation_container.perform();

        Some((mapped_pages, physical))
    }

    fn as_param(&self, allocation: &MailboxAllocation) -> u64 {
        let (_page, address) = allocation.as_ref().unwrap();
        address.as_u64()
    }
}

/// An output of a command.
///
/// This can be either `()` for commands that produce no output,
/// `u64` for commands that produce immediate input or
/// `MappedPages` for commands that write to a mailbox.
pub(super) trait OutputParameter {
    /// Possibly allocate.
    ///
    /// This will do if the output is not a mailbox.
    fn allocate() -> MailboxAllocation;

    /// Parse the result of a command's execution.
    fn from_result(value: u64, output_allocation: MailboxAllocation) -> Self;
}

impl OutputParameter for () {
    fn allocate() -> MailboxAllocation {
        None
    }

    fn from_result(_value: u64, output_allocation: MailboxAllocation) -> Self {
        // one could think that value == 0, but that's not always the case
        assert!(output_allocation.is_none());
        ()
    }
}
impl OutputParameter for u64 {
    fn allocate() -> MailboxAllocation {
        None
    }

    fn from_result(value: u64, output_allocation: MailboxAllocation) -> Self {
        assert!(output_allocation.is_none());
        value
    }
}
impl OutputParameter for utils::MappedPages {
    fn allocate() -> MailboxAllocation {
        let mut operation_container = Operations::default();
        let (mapped_pages, physical) = utils::create_cont_mapping_with_dma_flags(1).expect("").fetch_in_addr().expect("");
        let data = utils::start_page_as_mut_ptr::<u8>(mapped_pages.into_range().start);

        operation_container.add_operation(Box::new(FillOperation {}), OperationArgs::Fill(0u8, data, PAGE_SIZE));

        operation_container.perform();

        Some((mapped_pages, physical))
    }

    fn from_result(_value: u64, output_allocation: MailboxAllocation) -> Self {
        let (page, _physical) = output_allocation.unwrap();
        // one could think that value == physical.value() but that's not the case
        page
    }
}

impl<'a> CommandInterface<'a> {
    pub(super) fn new(config_regs: &'a mut utils::MappedPages) -> Result<Self, &'static str> {
        let hcr = config_regs.as_type_mut::<Hcr>(HCR_BASE)?;

        Ok(Self { hcr, exp_toggle: 1 })
    }

    /// Post a command and wait for its completion.
    ///
    /// Input and output can be either `()` (for opcodes that take no input or
    /// give no output), bytes / pages (for opcodes that read from or write to
    /// mailboxes or u64 (for opcodes the operate on immediate values).
    ///
    /// ## Safety
    ///
    /// This function does not check whether the specified opcode takes the
    /// provided type of input or output.
    pub(super) fn execute_command<M, I, O>(&mut self, opcode: Opcode, opcode_modifier: M, input: I, input_modifier: u32) -> Result<O, ReturnStatus>
    where
        M: OpcodeModifier,
        I: InputParameter,
        O: OutputParameter,
    {
        // TODO: timeout
        trace!("executing command: {opcode:?}");

        // wait until the previous command is done
        while self.is_pending() {
            trace!("wait until the previous command is done");
        }

        // allocate memory
        let input_allocation = input.allocate();
        let input_param = input.as_param(&input_allocation);
        let output_allocation = O::allocate();
        let output_param = if let Some((_, output_address)) = output_allocation {
            output_address.as_u64()
        } else {
            0
        };
        // post the command
        let ptr = self.hcr as *mut Hcr as *mut u32;
        self.hcr.in_param_h.set(((input_param >> 32) as u32).to_be());
        self.hcr.in_param_l.set((input_param as u32).to_be());
        self.hcr.in_mod.set(input_modifier.to_be());
        self.hcr.out_param_h.set(((output_param >> 32) as u32).to_be());
        self.hcr.out_param_l.set((output_param as u32).to_be());
        self.hcr.token.set((POLL_TOKEN << 16).to_be());
        compiler_fence(Ordering::SeqCst);
        let status_opcode = (1 << HCR_GO_BIT)
            | (self.exp_toggle << HCR_T_BIT)
            | (0 << HCR_E_BIT) // TODO: event
            | ((opcode_modifier.get() as u32) << HCR_OPMOD_SHIFT)
            | opcode as u16 as u32;
        self.hcr.status_opcode.set(status_opcode.to_be());
        self.exp_toggle ^= 1;

        trace!("polling for completion");
        // poll for it
        while self.is_pending() {}

        // check the status
        let status_opcode = u32::from_be(self.hcr.status_opcode.get());
        let status = ReturnStatus::from_repr(status_opcode >> 24).expect("return status invalid");
        trace!("status: {status:?}");
        let out_param_h = u32::from_be(self.hcr.out_param_h.get()) as u64;
        let out_param_l = u32::from_be(self.hcr.out_param_l.get()) as u64;
        let out_param: u64 = out_param_h << 32 | out_param_l ;
        trace!("out_param: 0x{out_param:x}");
        match status {
            // on success, return the result
            ReturnStatus::Ok => Ok(O::from_result(out_param, output_allocation)),
            // else, return the status
            err => Err(err),
        }
    }

    fn is_pending(&self) -> bool {
        let status = u32::from_be(self.hcr.status_opcode.get());
        status & (1 << HCR_GO_BIT) != 0 || (status & (1 << HCR_T_BIT)) == self.exp_toggle
    }
}

#[repr(u32)]
#[derive(Debug, FromRepr, IntoStaticStr)]
pub(super) enum ReturnStatus {
    // general
    Ok = 0x00,
    InternalErr = 0x01,
    BadOp = 0x02,
    BadParam = 0x03,
    BadSysState = 0x04,
    BadResource = 0x05,
    ResourceBusy = 0x06,
    ExceedLim = 0x08,
    BadResState = 0x09,
    BadIndex = 0x0a,
    BadNvmem = 0x0b,
    IcmError = 0x0c,
    BadPerm = 0x0d,

    // QP state
    BadQpState = 0x10,

    // TPT
    RegBound = 0x21,

    // MAD
    BadPkt = 0x30,

    // CQ
    BadSize = 0x40,
}
