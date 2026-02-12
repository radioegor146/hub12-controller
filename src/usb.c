#include "usb.h"

#include <hardware/irq.h>
#include <hardware/resets.h>
#include <hardware/structs/usb.h>
#include <hardware/structs/usb_dpram.h>
#include <pico/stdio.h>
#include <string.h>

#include "hub12.h"

#define USB_HW_SET ((usb_hw_t*)hw_set_alias_untyped(usb_hw))
#define USB_HW_CLEAR ((usb_hw_t*)hw_clear_alias_untyped(usb_hw))

#define EP0_IN_ADDR (USB_DIR_IN | 0)
#define EP0_OUT_ADDR (USB_DIR_OUT | 0)
#define EP1_OUT_ADDR (USB_DIR_OUT | 1)
#define EP2_OUT_ADDR (USB_DIR_OUT | 2)

static const USBEndpointDescriptor kEP0Out = {
    .bLength = sizeof(USBEndpointDescriptor),
    .bDescriptorType = USB_DT_ENDPOINT,
    .bEndpointAddress = EP0_OUT_ADDR,
    .bmAttributes = USB_TRANSFER_TYPE_CONTROL,
    .wMaxPacketSize = 64,
    .bInterval = 0};

static const USBEndpointDescriptor kEP0In = {
    .bLength = sizeof(USBEndpointDescriptor),
    .bDescriptorType = USB_DT_ENDPOINT,
    .bEndpointAddress = EP0_IN_ADDR,
    .bmAttributes = USB_TRANSFER_TYPE_CONTROL,
    .wMaxPacketSize = 64,
    .bInterval = 0};

static const USBEndpointDescriptor kEP1Out = {
    .bLength = sizeof(USBEndpointDescriptor),
    .bDescriptorType = USB_DT_ENDPOINT,
    .bEndpointAddress = EP1_OUT_ADDR,
    .bmAttributes = USB_TRANSFER_TYPE_BULK,
    .wMaxPacketSize = 64,
    .bInterval = 0};

static const USBEndpointDescriptor kEP2Out = {
    .bLength = sizeof(USBEndpointDescriptor),
    .bDescriptorType = USB_DT_ENDPOINT,
    .bEndpointAddress = EP2_OUT_ADDR,
    .bmAttributes = USB_TRANSFER_TYPE_BULK,
    .wMaxPacketSize = 64,
    .bInterval = 0};

static USBDeviceDescriptor kDeviceDescriptor = {
    .bLength = sizeof(USBDeviceDescriptor),
    .bDescriptorType = USB_DT_DEVICE,
    .bcdUSB = 0x0110,
    .bDeviceClass = 0xFF,
    .bDeviceSubClass = 0x13,
    .bDeviceProtocol = 0,
    .bMaxPacketSize0 = 64,
    .idVendor = 0xE146,
    .idProduct = 0x1337,
    .bcdDevice = 0,
    .iManufacturer = 1,
    .iProduct = 2,
    .iSerialNumber = 0,
    .bNumConfigurations = 1};

static USBInterfaceDescriptor kInterfaceDescription = {
    .bLength = sizeof(USBInterfaceDescriptor),
    .bDescriptorType = USB_DT_INTERFACE,
    .bInterfaceNumber = 0,
    .bAlternateSetting = 0,
    .bNumEndpoints = 2,
    .bInterfaceClass = 0xff,
    .bInterfaceSubClass = 0,
    .bInterfaceProtocol = 0,
    .iInterface = 0};

static USBConfigurationDescriptor kConfigurationDescriptor = {
    .bLength = sizeof(USBConfigurationDescriptor),
    .bDescriptorType = USB_DT_CONFIG,
    .wTotalLength = sizeof(kConfigurationDescriptor) +
                    sizeof(kInterfaceDescription) + sizeof(kEP1Out) +
                    sizeof(kEP2Out),
    .bNumInterfaces = 1,
    .bConfigurationValue = 1,
    .iConfiguration = 0,
    .bmAttributes = 0xc0,
    .bMaxPower = 0x32};

static const uint8_t kLangDescriptor[] = {4, 0x03, 0x09, 0x04};

static const uint8_t* kDescriptorStrings[] = {
    (const uint8_t*)"B4CKSP4CE", (const uint8_t*)"HUB12 LED controller"};

static void USBEP0OutHandler(uint8_t* buffer, uint16_t length);
static void USBEP0InHandler(uint8_t* buffer, uint16_t length);
static void USBEP1OutHandler(uint8_t* buffer, uint16_t length);
static void USBEP2OutHandler(uint8_t* buffer, uint16_t length);

static USBDeviceConfiguration kDeviceConfiguration = {
    .device_descriptor = &kDeviceDescriptor,
    .interface_descriptor = &kInterfaceDescription,
    .config_descriptor = &kConfigurationDescriptor,
    .lang_descriptor = kLangDescriptor,
    .descriptor_strings = kDescriptorStrings,
    .endpoints = {{
                      .descriptor = &kEP0Out,
                      .handler = &USBEP0OutHandler,
                      .endpoint_control = NULL,
                      .buffer_control = &usb_dpram->ep_buf_ctrl[0].out,
                      .data_buffer = &usb_dpram->ep0_buf_a[0],
                  },
                  {
                      .descriptor = &kEP0In,
                      .handler = &USBEP0InHandler,
                      .endpoint_control = NULL,
                      .buffer_control = &usb_dpram->ep_buf_ctrl[0].in,
                      .data_buffer = &usb_dpram->ep0_buf_a[0],
                  },
                  {
                      .descriptor = &kEP1Out,
                      .handler = &USBEP1OutHandler,
                      .endpoint_control = &usb_dpram->ep_ctrl[0].out,
                      .buffer_control = &usb_dpram->ep_buf_ctrl[1].out,
                      .data_buffer = &usb_dpram->epx_data[0 * 64],
                  },
                  {
                      .descriptor = &kEP2Out,
                      .handler = &USBEP2OutHandler,
                      .endpoint_control = &usb_dpram->ep_ctrl[1].out,
                      .buffer_control = &usb_dpram->ep_buf_ctrl[2].out,
                      .data_buffer = &usb_dpram->epx_data[1 * 64],
                  }}};

static uint8_t device_address = 0;
static bool should_set_address = false;
static bool configured = false;
static uint8_t ep0_buf[64];

static inline uint32_t USBGetBufferOffset(volatile const uint8_t* buffer) {
  return (uint32_t)buffer ^ (uint32_t)usb_dpram;
}

static void USBEndpointInitialize(const USBEndpointConfiguration* endpoint) {
  if (!endpoint->endpoint_control) {
    return;
  }

  uint32_t dpram_offset = USBGetBufferOffset(endpoint->data_buffer);
  uint32_t reg =
      EP_CTRL_ENABLE_BITS | EP_CTRL_INTERRUPT_PER_BUFFER |
      (endpoint->descriptor->bmAttributes << EP_CTRL_BUFFER_TYPE_LSB) |
      dpram_offset;
  *endpoint->endpoint_control = reg;
}

static void USBEndpointsInitialize() {
  const USBEndpointConfiguration* endpoints = kDeviceConfiguration.endpoints;
  for (int i = 0; i < USB_NUM_ENDPOINTS; i++) {
    if (endpoints[i].descriptor && endpoints[i].handler) {
      USBEndpointInitialize(&endpoints[i]);
    }
  }
}

void USBInitialize() {
  reset_unreset_block_num_wait_blocking(RESET_USBCTRL);
  memset(usb_dpram, 0, sizeof(*usb_dpram));
  irq_set_enabled(USBCTRL_IRQ, true);
  irq_set_priority(USBCTRL_IRQ, PICO_LOWEST_IRQ_PRIORITY);

  usb_hw->muxing = USB_USB_MUXING_TO_PHY_BITS | USB_USB_MUXING_SOFTCON_BITS;
  usb_hw->pwr =
      USB_USB_PWR_VBUS_DETECT_BITS | USB_USB_PWR_VBUS_DETECT_OVERRIDE_EN_BITS;
  usb_hw->main_ctrl = USB_MAIN_CTRL_CONTROLLER_EN_BITS;
  usb_hw->sie_ctrl = USB_SIE_CTRL_EP0_INT_1BUF_BITS;
  usb_hw->inte = USB_INTS_BUFF_STATUS_BITS | USB_INTS_BUS_RESET_BITS |
                 USB_INTS_SETUP_REQ_BITS;

  USBEndpointsInitialize();

  USB_HW_SET->sie_ctrl = USB_SIE_CTRL_PULLUP_EN_BITS;
}

static void USBDoBusReset(void) {
  device_address = 0;
  should_set_address = false;
  usb_hw->dev_addr_ctrl = 0;
  configured = false;
}

static void USBHandleEndpointBufferDone(USBEndpointConfiguration* endpoint) {
  uint32_t buffer_control = *endpoint->buffer_control;
  uint16_t len = buffer_control & USB_BUF_CTRL_LEN_MASK;
  endpoint->handler((uint8_t*)endpoint->data_buffer, len);
}

static void USBHandlerBufferDone(uint endpoint_number, bool in) {
  uint8_t endpoint_address = endpoint_number | (in ? USB_DIR_IN : 0);
  for (uint i = 0; i < USB_NUM_ENDPOINTS; i++) {
    USBEndpointConfiguration* endpoint = &kDeviceConfiguration.endpoints[i];
    if (!endpoint->descriptor || !endpoint->handler) {
      continue;
    }
    if (endpoint->descriptor->bEndpointAddress == endpoint_address) {
      USBHandleEndpointBufferDone(endpoint);
      return;
    }
  }
}

static void USBHandleBufferStatusIRQ() {
  uint32_t buffers = usb_hw->buf_status;
  uint32_t remaining_buffers = buffers;

  uint bit = 1u;
  for (uint i = 0; remaining_buffers && i < USB_NUM_ENDPOINTS * 2; i++) {
    if (remaining_buffers & bit) {
      USB_HW_CLEAR->buf_status = bit;
      USBHandlerBufferDone(i >> 1u, !(i & 1u));
      remaining_buffers &= ~bit;
    }
    bit <<= 1u;
  }
}

static USBEndpointConfiguration* USBGetEndpointConfiguration(uint8_t address) {
  USBEndpointConfiguration* endpoints = kDeviceConfiguration.endpoints;
  for (int i = 0; i < USB_NUM_ENDPOINTS; i++) {
    if (endpoints[i].descriptor &&
        (endpoints[i].descriptor->bEndpointAddress == address)) {
      return &endpoints[i];
    }
  }
  return NULL;
}

static inline bool USBEndpointIsTX(USBEndpointConfiguration* endpoint) {
  return endpoint->descriptor->bEndpointAddress & USB_DIR_IN;
}

static void USBStartTransfer(USBEndpointConfiguration* ep, uint8_t* buffer,
                             uint16_t length) {
  assert(length <= 64);
  uint32_t val = length | USB_BUF_CTRL_AVAIL;

  if (USBEndpointIsTX(ep)) {
    memcpy((void*)ep->data_buffer, (void*)buffer, length);
    val |= USB_BUF_CTRL_FULL;
  }

  val |= ep->next_pid ? USB_BUF_CTRL_DATA1_PID : USB_BUF_CTRL_DATA0_PID;
  ep->next_pid ^= 1u;

  *ep->buffer_control = val;
}

static void USBAcknowledgeOutRequest() {
  USBStartTransfer(USBGetEndpointConfiguration(EP0_IN_ADDR), NULL, 0);
}

static void USBHandleSetDeviceAddress(volatile USBSetupPacket* packet) {
  device_address = (packet->wValue & 0xff);
  should_set_address = true;
  USBAcknowledgeOutRequest();
}

static void USBHandleSetDeviceConfiguration(
    __unused volatile USBSetupPacket* packet) {
  USBAcknowledgeOutRequest();
  configured = true;
}

static void USBHandleGetDeviceDescriptor(volatile USBSetupPacket* packet) {
  const USBDeviceDescriptor* descriptor =
      kDeviceConfiguration.device_descriptor;
  USBEndpointConfiguration* endpoint = USBGetEndpointConfiguration(EP0_IN_ADDR);
  endpoint->next_pid = 1;
  USBStartTransfer(endpoint, (uint8_t*)descriptor,
                   MIN(sizeof(USBDeviceDescriptor), packet->wLength));
}

static void USBHandleGetConfigurationDescriptor(volatile USBSetupPacket* pkt) {
  uint8_t* buffer = &ep0_buf[0];

  const USBConfigurationDescriptor* d = kDeviceConfiguration.config_descriptor;
  memcpy((void*)buffer, d, sizeof(USBConfigurationDescriptor));
  buffer += sizeof(USBConfigurationDescriptor);

  if (pkt->wLength >= d->wTotalLength) {
    memcpy((void*)buffer, kDeviceConfiguration.interface_descriptor,
           sizeof(USBInterfaceDescriptor));
    buffer += sizeof(USBInterfaceDescriptor);
    const USBEndpointConfiguration* endpoint = kDeviceConfiguration.endpoints;

    for (uint i = 2; i < USB_NUM_ENDPOINTS; i++) {
      if (endpoint[i].descriptor) {
        memcpy((void*)buffer, endpoint[i].descriptor,
               sizeof(USBEndpointDescriptor));
        buffer += sizeof(USBEndpointDescriptor);
      }
    }
  }

  uint32_t length = (uint32_t)buffer - (uint32_t)&ep0_buf[0];
  USBStartTransfer(USBGetEndpointConfiguration(EP0_IN_ADDR), &ep0_buf[0],
                   MIN(length, pkt->wLength));
}

static uint8_t USBPrepareStringDescriptor(const unsigned char* string) {
  uint8_t bLength = 2 + (strlen((const char*)string) * 2);
  static const uint8_t bDescriptorType = 0x03;

  volatile uint8_t* buffer = &ep0_buf[0];
  *buffer++ = bLength;
  *buffer++ = bDescriptorType;

  uint8_t c;

  do {
    c = *string++;
    *buffer++ = c;
    *buffer++ = 0;
  } while (c != '\0');

  return bLength;
}

static void USBHandleGetStringDescriptor(volatile USBSetupPacket* packet) {
  uint8_t i = packet->wValue & 0xff;
  uint8_t length;

  if (i == 0) {
    length = 4;
    memcpy(&ep0_buf[0], kDeviceConfiguration.lang_descriptor, length);
  } else {
    length = USBPrepareStringDescriptor(
        kDeviceConfiguration.descriptor_strings[i - 1]);
  }

  USBStartTransfer(USBGetEndpointConfiguration(EP0_IN_ADDR), &ep0_buf[0],
                   MIN(length, packet->wLength));
}

static void USBHandleGetStatus() {
  uint8_t status[2] = {0, 0};
  USBStartTransfer(USBGetEndpointConfiguration(EP0_IN_ADDR), status,
                   sizeof(status));
}

static void USBHandleSetupPacket() {
  volatile USBSetupPacket* packet =
      (volatile USBSetupPacket*)&usb_dpram->setup_packet;
  uint8_t req_direction = packet->bmRequestType;
  uint8_t req = packet->bRequest;

  USBGetEndpointConfiguration(EP0_IN_ADDR)->next_pid = 1u;

  if (req_direction == USB_DIR_OUT) {
    if (req == USB_REQUEST_SET_ADDRESS) {
      USBHandleSetDeviceAddress(packet);
    } else if (req == USB_REQUEST_SET_CONFIGURATION) {
      USBHandleSetDeviceConfiguration(packet);
    } else {
      USBAcknowledgeOutRequest();
    }
  } else if (req_direction == USB_DIR_IN) {
    if (req == USB_REQUEST_GET_STATUS) {
      USBHandleGetStatus();
    } else if (req == USB_REQUEST_GET_DESCRIPTOR) {
      uint16_t descriptor_type = packet->wValue >> 8;

      if (descriptor_type == USB_DT_DEVICE) {
        USBHandleGetDeviceDescriptor(packet);
      } else if (descriptor_type == USB_DT_CONFIG) {
        USBHandleGetConfigurationDescriptor(packet);
      } else if (descriptor_type == USB_DT_STRING) {
        USBHandleGetStringDescriptor(packet);
      }
    }
  }
}

void isr_usbctrl(void) {
  uint32_t status = usb_hw->ints;
  uint32_t handled = 0;

  if (status & USB_INTS_SETUP_REQ_BITS) {
    handled |= USB_INTS_SETUP_REQ_BITS;
    USB_HW_CLEAR->sie_status = USB_SIE_STATUS_SETUP_REC_BITS;
    USBHandleSetupPacket();
  }

  if (status & USB_INTS_BUFF_STATUS_BITS) {
    handled |= USB_INTS_BUFF_STATUS_BITS;
    USBHandleBufferStatusIRQ();
  }

  if (status & USB_INTS_BUS_RESET_BITS) {
    handled |= USB_INTS_BUS_RESET_BITS;
    USB_HW_CLEAR->sie_status = USB_SIE_STATUS_BUS_RESET_BITS;
    USBDoBusReset();
  }
}

static void USBEP0OutHandler(uint8_t* buffer, uint16_t length) {}

static void USBEP0InHandler(uint8_t* buffer, uint16_t length) {
  if (should_set_address) {
    usb_hw->dev_addr_ctrl = device_address;
    should_set_address = false;
  } else {
    USBStartTransfer(USBGetEndpointConfiguration(EP0_OUT_ADDR), NULL, 0);
  }
}

uint32_t buffer_position = 0;

static void USBEP1OutHandler(uint8_t* buffer, uint16_t length) {
  while (length) {
    uint8_t* led_buffer = Hub12GetAvailableBuffer();
    uint32_t read_length = MIN(BUFFER_SIZE - buffer_position, length);
    memcpy(led_buffer + buffer_position, buffer, read_length);
    buffer_position += read_length;
    if (buffer_position == BUFFER_SIZE) {
      Hub12PushBuffer();
      buffer_position = 0;
    }
    buffer += read_length;
    length -= read_length;
  }
  USBStartTransfer(USBGetEndpointConfiguration(EP1_OUT_ADDR), NULL, 64);
}

static void USBEP2OutHandler(uint8_t* buffer, uint16_t length) {
  if (length > 0) {
    buffer_position = 0;
  }
  USBStartTransfer(USBGetEndpointConfiguration(EP2_OUT_ADDR), NULL, 64);
}

void USBWaitForConfiguration() {
  while (!configured) {
    tight_loop_contents();
  }
  USBStartTransfer(USBGetEndpointConfiguration(EP1_OUT_ADDR), NULL, 64);
  USBStartTransfer(USBGetEndpointConfiguration(EP2_OUT_ADDR), NULL, 64);
}