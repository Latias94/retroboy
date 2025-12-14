/// Kind of instruction currently being executed via the micro-step API.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum MicroInstrKind {
    None,
    CallA16,
    Ret,
    RetCond,
    JpA16,
    JpCondA16,
    LdSpA16,
    Rst,
    Jr,
    JrCond,
    CallCond,
    /// LD (HL),r (8 T-cycles when r != (HL)).
    LdHlFromR,
    /// LD r,(HL) (8 T-cycles when r != (HL)).
    LdRFromHl,
    /// LD (a16),A
    LdAToA16,
    /// LD A,(a16)
    LdAFromA16,
    /// LDH A,(a8)
    LdhAFromA8,
    /// LDH (a8),A
    LdhAToA8,
    /// LD A,(C)
    LdhAFromC,
    /// LD (C),A
    LdhAToC,
}
