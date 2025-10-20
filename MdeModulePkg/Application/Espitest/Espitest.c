/**
  Simple UEFI application that mirrors the behavior of the provided Rust eSPI test.
  It performs volatile MMIO reads/writes to the same addresses and prints values to
  the UEFI console. This is intended as a test application for EDK II.

  License: same dual-license note as original.
*/

#include <Uefi.h>
#include <Library/UefiLib.h>
#include <Library/UefiBootServicesTableLib.h>
#include <Library/PrintLib.h>
#include <Library/DebugLib.h>

#define ESPI_BASE_ADDR     0xBC100000u
//#define ESPI_BASE_ADDR     0x70000000u

// MMIO addresses from the Rust source
#define DN_TXHDR_0_ADDR       (ESPI_BASE_ADDR + 0x0000u)
#define DN_TXHDR_1_ADDR       (ESPI_BASE_ADDR + 0x0004u)
#define DN_TXHDR_2_ADDR       (ESPI_BASE_ADDR + 0x0008u)
#define DN_TXDATA_PORT_ADDR   (ESPI_BASE_ADDR + 0x000Cu)
#define UP_RXHDR_0_ADDR       (ESPI_BASE_ADDR + 0x0010u)
#define UP_RXHDR_1_ADDR       (ESPI_BASE_ADDR + 0x0014u)
#define UP_RXDATA_PORT_ADDR   (ESPI_BASE_ADDR + 0x0018u)
#define MISC_CONTROL_REG_ADDR (ESPI_BASE_ADDR + 0x0020u)
#define MASTER_CAP_ADDR       (ESPI_BASE_ADDR + 0x002Cu)
#define GLOBAL_CONTROL_0_ADDR (ESPI_BASE_ADDR + 0x0030u)
#define GLOBAL_CONTROL_1_ADDR (ESPI_BASE_ADDR + 0x0034u)
#define SLAVE0_DECODE_EN_ADDR (ESPI_BASE_ADDR + 0x0040u)
#define SLAVE0_CONFIG_ADDR    (ESPI_BASE_ADDR + 0x0068u)
#define SLAVE0_INT_EN_ADDR    (ESPI_BASE_ADDR + 0x006Cu)
#define SLAVE0_INT_STS_ADDR   (ESPI_BASE_ADDR + 0x0070u)
#define SLAVE0_RXMSG_HDR0_ADDR (ESPI_BASE_ADDR + 0x0074u)
#define SLAVE0_RXMSG_HDR1_ADDR (ESPI_BASE_ADDR + 0x0078u)
#define SLAVE0_RXMSG_DATA_PORT_ADDR (ESPI_BASE_ADDR + 0x007Cu)
#define SLAVE0_RXVW_ADDR      (ESPI_BASE_ADDR + 0x009Cu)
#define SLAVE0_RXVW_DATA_ADDR (ESPI_BASE_ADDR + 0x00A0u)
#define SLAVE0_RXVW_INDEX_ADDR (ESPI_BASE_ADDR + 0x00A4u)
#define SLAVE0_RXVW_MISC_CNTL_ADDR (ESPI_BASE_ADDR + 0x00A8u)
#define ESPI_TRAN_CONTROL_ADDR (ESPI_BASE_ADDR + 0x0124u)

// Masks / events
#define CMD_STATUS_MASK      (1u << 3)
#define DNCMD_INT_STATUS     (1u << 28)
#define RXMSG_INT_STATUS     (1u << 29)
#define RXOOB_INT_STATUS     (1u << 30)

STATIC
UINT32
MmioRead32 (
  IN UINTN Address
  )
{
  return *((volatile UINT32 *) (UINTN) Address);
}

STATIC
VOID
MmioWrite32 (
  IN UINTN Address,
  IN UINT32 Value
  )
{
  *((volatile UINT32 *) (UINTN) Address) = Value;
}

STATIC
VOID
StartCmd(VOID)
{
  UINT32 val = MmioRead32(DN_TXHDR_0_ADDR) | CMD_STATUS_MASK;
  MmioWrite32(DN_TXHDR_0_ADDR, val);
}

STATIC
VOID
WaitCmdDone(VOID)
{
  while ((MmioRead32(DN_TXHDR_0_ADDR) & CMD_STATUS_MASK) != 0) {
    CpuPause();
  }
}

STATIC
EFI_STATUS
MapEspiRegion(VOID)
{
  EFI_STATUS Status;
  EFI_PHYSICAL_ADDRESS Base = (EFI_PHYSICAL_ADDRESS)ESPI_BASE_ADDR;
  UINTN Pages = 1; // 4 KiB

  // Try to reserve the MMIO region at ESPI_BASE_ADDR to prevent other allocators
  Status = gBS->AllocatePages(AllocateAddress, EfiMemoryMappedIO, Pages, &Base);
  if (EFI_ERROR(Status)) {
    DEBUG((DEBUG_INFO, "MapEspiRegion: AllocatePages failed: %r\n", Status));
    return Status;
  }

  if (Base != (EFI_PHYSICAL_ADDRESS)ESPI_BASE_ADDR) {
    DEBUG((DEBUG_INFO, "MapEspiRegion: allocated base %p not expected %p\n", (VOID*) (UINTN) Base, (VOID*) (UINTN) ESPI_BASE_ADDR));
    // Not fatal, but warn
  } else {
    DEBUG((DEBUG_INFO, "MapEspiRegion: reserved ESPI MMIO at %p (4 KiB)\n", (VOID*) (UINTN) Base));
  }

  return EFI_SUCCESS;
}

STATIC
VOID
ClearAllInt(VOID)
{
  MmioWrite32(SLAVE0_INT_STS_ADDR, 0xFFFFFFFF);
  // Clear any pending OOB data
  MmioWrite32(UP_RXHDR_0_ADDR, CMD_STATUS_MASK);
}

STATIC
VOID
WaitEvent(UINT32 Event)
{
  while ((MmioRead32(SLAVE0_INT_STS_ADDR) & Event) == 0) {
    CpuPause();
  }
  // Clear the event
  MmioWrite32(SLAVE0_INT_STS_ADDR, Event);
}

STATIC
VOID
ClearEvent(UINT32 Event)
{
  MmioWrite32(SLAVE0_INT_STS_ADDR, Event);
}

STATIC
VOID
SetConfiguration(UINT16 Reg, UINT32 Val)
{
  WaitCmdDone();
  ClearEvent(DNCMD_INT_STATUS);
  MmioWrite32(DN_TXHDR_0_ADDR, ((UINT32)(Reg & 0xff) << 16));
  MmioWrite32(DN_TXHDR_1_ADDR, Val);
  MmioWrite32(DN_TXHDR_2_ADDR, 0);
  MmioWrite32(DN_TXDATA_PORT_ADDR, 0);
  StartCmd();
  WaitEvent(DNCMD_INT_STATUS);
  WaitCmdDone();
}

STATIC
UINT32
GetConfiguration(UINT16 Reg)
{
  WaitCmdDone();
  ClearEvent(DNCMD_INT_STATUS);
  MmioWrite32(DN_TXHDR_1_ADDR, 0);
  MmioWrite32(DN_TXHDR_2_ADDR, 0);
  MmioWrite32(DN_TXDATA_PORT_ADDR, 0);
  MmioWrite32(DN_TXHDR_0_ADDR, 0x1);
  MmioWrite32(DN_TXHDR_0_ADDR, 0x1 | ((UINT32)(Reg & 0xff) << 16));
  StartCmd();
  WaitEvent(DNCMD_INT_STATUS);
  WaitCmdDone();
  return MmioRead32(DN_TXHDR_1_ADDR);
}

STATIC
VOID
SendInbandReset(VOID)
{
  WaitCmdDone();
  MmioWrite32(DN_TXHDR_1_ADDR, 0);
  MmioWrite32(DN_TXHDR_2_ADDR, 0);
  MmioWrite32(DN_TXDATA_PORT_ADDR, 0);
  MmioWrite32(DN_TXHDR_0_ADDR, 0x2);
  StartCmd();
  WaitCmdDone();

  // Set back to default per updated Rust
  MmioWrite32(SLAVE0_CONFIG_ADDR, 0xC0000008);
}

STATIC
VOID
InitEspiBus(VOID)
{
  UINT32 tran_control;
  tran_control = MmioRead32(ESPI_TRAN_CONTROL_ADDR) & ~(1u << 20);
  MmioWrite32(ESPI_TRAN_CONTROL_ADDR, tran_control);

  // Issue SW_RST
  DEBUG((DEBUG_INFO, "Assert SW_RST\n"));
  MmioWrite32(GLOBAL_CONTROL_1_ADDR, 0x1);
  DEBUG((DEBUG_INFO, "Deassert SW_RST\n"));
  MmioWrite32(GLOBAL_CONTROL_1_ADDR, 0x0);
  (void)MmioRead32(SLAVE0_RXVW_ADDR);
  MmioWrite32(SLAVE0_RXVW_ADDR, 0xFFFF6F00);
  DEBUG((DEBUG_INFO, "MASTER_CAP: %08x\n", MmioRead32(MASTER_CAP_ADDR)));
  MmioWrite32(GLOBAL_CONTROL_0_ADDR, 0x7B | (0x1400 << 8) | (0x3F << 24));
  MmioWrite32(SLAVE0_RXVW_MISC_CNTL_ADDR, 0xF);
  MmioWrite32(GLOBAL_CONTROL_1_ADDR, (1u<<20) | (0x2FFu << 8) | (1u << 1));
  MmioWrite32(SLAVE0_CONFIG_ADDR, 0xC0000008);
}

STATIC
VOID
SetVWire(UINT8 Index, UINT8 Data)
{
  // Two VWire events packed into a single 32-bit write as in Rust
  WaitCmdDone();
  ClearEvent(DNCMD_INT_STATUS);
  MmioWrite32(DN_TXHDR_0_ADDR, 0x5 | ((2u << 8) & 0xFFFFFFFF)); // VWire, count = 2
  MmioWrite32(DN_TXHDR_1_ADDR, 0x0);
  MmioWrite32(DN_TXHDR_2_ADDR, 0x0);
  UINT32 packed = (((Data & 0xF0u) << 24) | ((Index & 0xFFu) << 16) | (Index & 0xFFu) | ((Data & 0xFFu) << 8));
  MmioWrite32(DN_TXDATA_PORT_ADDR, packed);
  StartCmd();
  WaitEvent(DNCMD_INT_STATUS);
  WaitCmdDone();
}

STATIC
VOID
PutOob(UINT8 Target, UINT8 MctpCode, UINT8 MctpBytes, CONST UINT8 *Data, UINTN Len)
{
  // Len is number of bytes in Data. We'll write 4-byte words to DN_TXDATA_PORT
  WaitCmdDone();
  ClearEvent(DNCMD_INT_STATUS);

  MmioWrite32(DN_TXHDR_0_ADDR, 6u | (0x21u << 8) | (0x3u << 20) | (11u << 24));
  MmioWrite32(DN_TXHDR_1_ADDR, (UINT32)Target | ((UINT32)MctpCode << 8) | ((UINT32)MctpBytes << 16));
  MmioWrite32(DN_TXHDR_2_ADDR, 0x12345678);

  // Write data in little-endian 32-bit words, pad with zeros if needed
  for (UINTN i = 0; i < Len; i += 4) {
    UINT8 b0 = (i + 0 < Len) ? Data[i + 0] : 0;
    UINT8 b1 = (i + 1 < Len) ? Data[i + 1] : 0;
    UINT8 b2 = (i + 2 < Len) ? Data[i + 2] : 0;
    UINT8 b3 = (i + 3 < Len) ? Data[i + 3] : 0;
    UINT32 word = (UINT32)b0 | ((UINT32)b1 << 8) | ((UINT32)b2 << 16) | ((UINT32)b3 << 24);
    DEBUG((DEBUG_INFO, "Writing value: %08x\n", word));
    MmioWrite32(DN_TXDATA_PORT_ADDR, word);
  }

  DEBUG((DEBUG_INFO, "Send the OOB command\n"));
  StartCmd();
  WaitEvent(DNCMD_INT_STATUS);
  WaitCmdDone();
  DEBUG((DEBUG_INFO, "OOB command complete\n"));
}

STATIC
UINT32
GetOob(VOID)
{
  while ((MmioRead32(UP_RXHDR_0_ADDR) & CMD_STATUS_MASK) != 0) {
    CpuPause();
  }
  UINT8 rx_len = (UINT8)(MmioRead32(UP_RXHDR_0_ADDR) >> 24);
  DEBUG((DEBUG_INFO, "RX OOB len: %02x\n", rx_len));
  UINT32 rxhdr1 = MmioRead32(UP_RXHDR_1_ADDR);
  DEBUG((DEBUG_INFO, "Target: %02x\n", (UINT8)(rxhdr1 & 0xFF)));
  DEBUG((DEBUG_INFO, "Opcode: %02x\n", (UINT8)((rxhdr1 >> 8) & 0xFF)));
  DEBUG((DEBUG_INFO, "Length: %02x\n", (UINT8)((rxhdr1 >> 16) & 0xFF)));

  while (rx_len > 4) {
    (void)MmioRead32(UP_RXDATA_PORT_ADDR);
    rx_len -= 4;
  }
  UINT32 data = MmioRead32(UP_RXDATA_PORT_ADDR);
  // Clear the RX data pending
  MmioWrite32(UP_RXHDR_0_ADDR, CMD_STATUS_MASK);
  return data;
}

STATIC
VOID
PeripheralWrite(UINT32 Addr, UINT32 Data)
{
  WaitCmdDone();
  ClearEvent(DNCMD_INT_STATUS);
  MmioWrite32(DN_TXHDR_0_ADDR, 4u | (1u << 8) | (0x3u << 20) | (0u << 24));
  MmioWrite32(DN_TXHDR_1_ADDR, Addr);
  MmioWrite32(DN_TXHDR_2_ADDR, 0);
  MmioWrite32(DN_TXDATA_PORT_ADDR, Data);
  StartCmd();
  WaitEvent(DNCMD_INT_STATUS);
  WaitCmdDone();
}

STATIC
UINT32
PeripheralRead(UINT32 Addr)
{
  WaitCmdDone();
  ClearEvent(DNCMD_INT_STATUS);
  DEBUG((DEBUG_INFO, "Sending downstream Read 32 request\n"));
  MmioWrite32(DN_TXHDR_0_ADDR, 4u | (0u << 8) | (0x3u << 20) | (7u << 24));
  MmioWrite32(DN_TXHDR_1_ADDR, Addr);
  MmioWrite32(DN_TXHDR_2_ADDR, 0);
  StartCmd();
  WaitEvent(DNCMD_INT_STATUS);
  WaitCmdDone();

  // Now wait for upstream message
  DEBUG((DEBUG_INFO, "Waiting for upstream message\n"));
  WaitEvent(RXMSG_INT_STATUS);
  UINT32 hdr0 = MmioRead32(SLAVE0_RXMSG_HDR0_ADDR);
  DEBUG((DEBUG_INFO, "Cycle Type: %02x\n", (UINT8)(hdr0 & 0xFF)));
  DEBUG((DEBUG_INFO, "Tag: %02x\n", (UINT8)((hdr0 >> 8) & 0xFF)));
  DEBUG((DEBUG_INFO, "Length: %02x\n", (UINT8)((hdr0 >> 16) & 0xFF)));
  DEBUG((DEBUG_INFO, "Data0: %02x\n", (UINT8)((hdr0 >> 24) & 0xFF)));

  return MmioRead32(SLAVE0_RXMSG_HDR1_ADDR);
}

STATIC
VOID
InitEspiDevice(VOID)
{
  // Skip bus init like the Rust version
  DEBUG((DEBUG_INFO, "Bus initialization\n"));
  InitEspiBus();
  ClearAllInt();
  MmioWrite32(SLAVE0_INT_EN_ADDR, 0);
  MmioWrite32(SLAVE0_DECODE_EN_ADDR, 0x4); // Enable port 80/60-64

  DEBUG((DEBUG_INFO, "Send Inband reset\n"));
  SendInbandReset();
  DEBUG((DEBUG_INFO, "Completed Inband reset\n"));

  DEBUG((DEBUG_INFO, "ESPI_TRAN_CONTROL %08x\n", MmioRead32(ESPI_TRAN_CONTROL_ADDR)));
  DEBUG((DEBUG_INFO, "DN_TXHDR_0 %08x\n", MmioRead32(DN_TXHDR_0_ADDR)));
  DEBUG((DEBUG_INFO, "MISC_CONTROL_REG %08x\n", MmioRead32(MISC_CONTROL_REG_ADDR)));
  DEBUG((DEBUG_INFO, "GLOBAL_CONTROL_0 %08x\n", MmioRead32(GLOBAL_CONTROL_0_ADDR)));
  DEBUG((DEBUG_INFO, "GLOBAL_CONTROL_1 %08x\n", MmioRead32(GLOBAL_CONTROL_1_ADDR)));
  DEBUG((DEBUG_INFO, "SLAVE0_DECODE_EN %08x\n", MmioRead32(SLAVE0_DECODE_EN_ADDR)));
  DEBUG((DEBUG_INFO, "SLAVE0_CONFIG %08x\n", MmioRead32(SLAVE0_CONFIG_ADDR)));
  DEBUG((DEBUG_INFO, "SLAVE0_INT_EN %08x\n", MmioRead32(SLAVE0_INT_EN_ADDR)));
  DEBUG((DEBUG_INFO, "SLAVE0_INT_STS %08x\n", MmioRead32(SLAVE0_INT_STS_ADDR)));
  DEBUG((DEBUG_INFO, "SLAVE0_RXVW_MISC_CNTL %08x\n", MmioRead32(SLAVE0_RXVW_MISC_CNTL_ADDR)));

  DEBUG((DEBUG_INFO, "GET_CONFIGURATION\n"));
  UINT32 gen_cap = GetConfiguration(0x8);
  DEBUG((DEBUG_INFO, "DEVICE_ID    %08x\n", GetConfiguration(0x4)));
  DEBUG((DEBUG_INFO, "GENERAL_CAP  %08x\n", gen_cap));
  DEBUG((DEBUG_INFO, "CHANNEL0_CAP %08x\n", GetConfiguration(0x10)));
  DEBUG((DEBUG_INFO, "CHANNEL1_CAP %08x\n", GetConfiguration(0x20)));
  DEBUG((DEBUG_INFO, "SLAVE0_INT_STS %08x\n", MmioRead32(SLAVE0_INT_STS_ADDR)));

  DEBUG((DEBUG_INFO, "Enable Alert mode and CRC checking\n"));
  SetConfiguration(0x8, gen_cap | (1u << 31) | (1u << 28));

  DEBUG((DEBUG_INFO, "Enable VW in SLAVE0_CONFIG\n"));
  MmioWrite32(SLAVE0_CONFIG_ADDR, 0xC0000004);
  SetConfiguration(0x20, 1); // Enable VW

  DEBUG((DEBUG_INFO, "Wait for VWire to go high\n"));
  for(;;) {
    if(GetConfiguration(0x20) & 0x1) {
      break;
    }
  }

  DEBUG((DEBUG_INFO, "Enabling 4 virtual wires\n"));
  SetConfiguration(0x20, 0x30003);

  DEBUG((DEBUG_INFO, "GENERAL_CAP  %08x\n", GetConfiguration(0x8)));
  DEBUG((DEBUG_INFO, "SLAVE0_CONFIG %08x\n", MmioRead32(SLAVE0_CONFIG_ADDR)));
  DEBUG((DEBUG_INFO, "SLAVE0_INT_EN %08x\n", MmioRead32(SLAVE0_INT_EN_ADDR)));
  DEBUG((DEBUG_INFO, "SLAVE0_INT_STS %08x\n", MmioRead32(SLAVE0_INT_STS_ADDR)));

  DEBUG((DEBUG_INFO, "De-asserting PLTRST and SUS_STS\n"));
  SetVWire(0x3, 0x22); // PLTRST de-assert

  // Enable OOB and Peripheral Channels
  MmioWrite32(SLAVE0_CONFIG_ADDR, 0xC000000E);
  SetConfiguration(0x10, 0x7115); // Peripheral 64 byte payload with 4K max read enable bit
  SetConfiguration(0x30,0x111); // OOB 64 byte transfer and enable bit

}

STATIC
VOID
RunEspiTest(VOID)
{
  DEBUG((DEBUG_INFO, "OOB write\n"));
  
  UINT8 OobData[16] = {
    0x1, // Source Address 
    0x1, // Header Version
    0x2, // Destination EID
    0x1, // Source EID
    0xD3, // Flags
    0xFE, // Command Data
    0,0,0,0,0,0,0,0,0,0 // Padding
  };

  PutOob(0x2, 0xf, 0x10, OobData, sizeof(OobData));

  DEBUG((DEBUG_INFO, "OOB read\n"));
  DEBUG((DEBUG_INFO, "SLAVE0_INT_STS %08x\n", MmioRead32(SLAVE0_INT_STS_ADDR)));
  UINT32 oob_response = GetOob();
  DEBUG((DEBUG_INFO, "OOB Response Data: %08x\n", oob_response));

  DEBUG((DEBUG_INFO, "Peripheral write\n"));
  PeripheralWrite(0x0, 0xDEADD00D);

  DEBUG((DEBUG_INFO, "Peripheral read\n"));
  UINT32 peripheral_data = PeripheralRead(0x0); 
  DEBUG((DEBUG_INFO, "Peripheral Read Data: %08x\n", peripheral_data));
}

/**
  UEFI application entry point
*/
EFI_STATUS
EFIAPI
_ModuleEntryPoint (
  IN EFI_HANDLE ImageHandle,
  IN EFI_SYSTEM_TABLE *SystemTable
  )
{
  EFI_STATUS Status = EFI_SUCCESS;
  DEBUG((DEBUG_INFO, "eSPI Test UEFI application\n"));

  // Reserve/map the eSPI MMIO region before touching it
  EFI_STATUS MapStatus = MapEspiRegion();
  if (EFI_ERROR(MapStatus)) {
    DEBUG((DEBUG_INFO, "Warning: could not reserve ESPI MMIO region (%r). Continuing anyway.\n", MapStatus));
  }

  InitEspiDevice();
  RunEspiTest();

  return Status;
}
