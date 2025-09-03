#include "usbd_audio.h"
#include "usbd_ctlreq.h"
#include "usbd_core.h"
#include <math.h>
#include "myfreertos.h"

extern void uart2_print(const char *str);
extern uint16_t i2s_dma_buf[AUDIO_DMA_BUF_SIZE];

/* define class_func */
static uint8_t USBD_AUDIO_Init(USBD_HandleTypeDef *pdev, uint8_t cfgidx);
static uint8_t USBD_AUDIO_DeInit(USBD_HandleTypeDef *pdev, uint8_t cfgidx);
static uint8_t USBD_AUDIO_Setup(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req);
static uint8_t *USBD_AUDIO_GetCfgDesc(uint16_t *length);
static uint8_t *USBD_AUDIO_GetDeviceQualifierDesc(uint16_t *length);
static uint8_t USBD_AUDIO_DataIn(USBD_HandleTypeDef *pdev, uint8_t epnum);
static uint8_t USBD_AUDIO_DataOut(USBD_HandleTypeDef *pdev, uint8_t epnum);
static uint8_t USBD_AUDIO_EP0_RxReady(USBD_HandleTypeDef *pdev);
static uint8_t USBD_AUDIO_EP0_TxReady(USBD_HandleTypeDef *pdev);
static uint8_t USBD_AUDIO_SOF(USBD_HandleTypeDef *pdev);
static uint8_t USBD_AUDIO_IsoINIncomplete(USBD_HandleTypeDef *pdev, uint8_t epnum);
static uint8_t USBD_AUDIO_IsoOutIncomplete(USBD_HandleTypeDef *pdev, uint8_t epnum);
static void    AUDIO_REQ_GetCurrent(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req);
static void    AUDIO_REQ_SetCurrent(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req);

/* define funcs_struct */
USBD_ClassTypeDef  USBD_AUDIO =
{
  USBD_AUDIO_Init,
  USBD_AUDIO_DeInit,
  USBD_AUDIO_Setup,
  USBD_AUDIO_EP0_TxReady,
  USBD_AUDIO_EP0_RxReady,
  USBD_AUDIO_DataIn,
  USBD_AUDIO_DataOut,
  USBD_AUDIO_SOF,
  USBD_AUDIO_IsoINIncomplete,
  USBD_AUDIO_IsoOutIncomplete,
  USBD_AUDIO_GetCfgDesc,
  USBD_AUDIO_GetCfgDesc,
  USBD_AUDIO_GetCfgDesc,
  USBD_AUDIO_GetDeviceQualifierDesc,
};

__ALIGN_BEGIN static uint8_t USBD_AUDIO_CfgDesc[] __ALIGN_END =
{
    /* Configuration Descriptor Head */
    0x09,                           // bLength
    0x02,                           // bDescriptorType = CONFIGURATION
    0x6D, 0x00,                     // wTotalLength = 109 bytes
    0x02,                           // bNumInterfaces = 2 (Audio Control + Audio Streaming)
    0x01,                           // bConfigurationValue
    0x00,                           // iConfiguration
    0x80,                           // bmAttributes: Bus powered
    0x32,                           // bMaxPower: 100mA

    /* -------- Audio Control Interface (Interface 0) -------- */
    0x09,                           // bLength
    0x04,                           // bDescriptorType = INTERFACE
    0x00,                           // bInterfaceNumber = 0
    0x00,                           // bAlternateSetting
    0x00,                           // bNumEndpoints
    0x01,                           // bInterfaceClass = AUDIO
    0x01,                           // bInterfaceSubClass = AUDIOCONTROL
    0x00,                           // bInterfaceProtocol
    0x00,                           // iInterface

    /* Class-Specific AC Interface Header Descriptor */
    0x09,                           // bLength
    0x24,                           // bDescriptorType = CS_INTERFACE
    0x01,                           // bDescriptorSubtype = HEADER
    0x00, 0x01,                     // bcdADC = 1.00
    0x27, 0x00,                     // wTotalLength = 30 bytes (AC descriptors)
    0x01,                           // bInCollection
    0x01,                           // baInterfaceNr(1) = 1 (AS interface)

    /* Input Terminal Descriptor (Microphone) */
    0x0C,                           // bLength
    0x24,                           // bDescriptorType = CS_INTERFACE
    0x02,                           // bDescriptorSubtype = INPUT_TERMINAL
    0x01,                           // bTerminalID
    0x01, 0x02,                     // wTerminalType = Microphone (0x0201)
    0x00,                           // bAssocTerminal
    0x01,                           // bNrChannels = 1 (Mono)
    0x00, 0x00,                     // wChannelConfig = Mono (no spatial config)
    0x00,                           // iChannelNames
    0x00,                           // iTerminal

    /* Feature Unit Descriptor */
    0x09,                           // bLength
    0x24,                           // bDescriptorType = CS_INTERFACE
    0x06,                           // bDescriptorSubtype = FEATURE_UNIT
    0x02,                           // bUnitID
    0x01,                           // bSourceID (Input Terminal = 1)
    0x01,                           // bControlSize = 1 byte per channel
    0x03,                           // bmaControls(0): Mute + volum Master channel
    0x00,                           // bmaControls(1): none for channel 1
    0x00,                           // iFeature

    /* Output Terminal Descriptor (USB Streaming) */
    0x09,                           // bLength
    0x24,                           // bDescriptorType = CS_INTERFACE
    0x03,                           // bDescriptorSubtype = OUTPUT_TERMINAL
    0x03,                           // bTerminalID
    0x01, 0x01,                     // wTerminalType = USB Streaming (0x0101)
    0x00,                           // bAssocTerminal
    0x01,                           // bSourceID (Feature Unit = 2)
    0x00,                           // iTerminal

    /* -------- Audio Streaming Interface (Interface 1) -------- */

    /* AS Interface Descriptor (Alt 0 - Zero Bandwidth) */
    0x09,                           // bLength
    0x04,                           // bDescriptorType = INTERFACE
    0x01,                           // bInterfaceNumber = 1
    0x00,                           // bAlternateSetting = 0
    0x00,                           // bNumEndpoints = 0
    0x01,                           // bInterfaceClass = AUDIO
    0x02,                           // bInterfaceSubClass = AUDIOSTREAMING
    0x00,                           // bInterfaceProtocol
    0x00,                           // iInterface

    /* AS Interface Descriptor (Alt 1 - Operational) */
    0x09,                           // bLength
    0x04,                           // bDescriptorType = INTERFACE
    0x01,                           // bInterfaceNumber = 1
    0x01,                           // bAlternateSetting = 1
    0x01,                           // bNumEndpoints = 1 (Iso IN)
    0x01,                           // bInterfaceClass = AUDIO
    0x02,                           // bInterfaceSubClass = AUDIOSTREAMING
    0x00,                           // bInterfaceProtocol
    0x00,                           // iInterface

    /* Class-Specific AS General Descriptor */
    0x07,                           // bLength
    0x24,                           // bDescriptorType = CS_INTERFACE
    0x01,                           // bDescriptorSubtype = AS_GENERAL
    0x03,                           // bTerminalLink (Output Terminal ID = 3)
    0x01,                           // bDelay
    0x01, 0x00,                     // wFormatTag = PCM

    /* Type I Format Type Descriptor */
    0x0B,                           // bLength
    0x24,                           // bDescriptorType = CS_INTERFACE
    0x02,                           // bDescriptorSubtype = FORMAT_TYPE
    0x01,                           // bFormatType = FORMAT_TYPE_I
    0x01,                           // bNrChannels = 1
    0x02,                           // bSubframeSize = 2 bytes per sample
    0x10,                           // bBitResolution = 16 bits
    0x01,                           // bSamFreqType = 1 (Discrete Sampling Frequency)
    0x80, 0xBB, 0x00,               // tSamFreq[1] = 48000 Hz (0x00BB80)

    /* Standard Endpoint Descriptor (Isochronous IN Audio Data Endpoint) */
    0x09,                           // bLength
    0x05,                           // bDescriptorType = ENDPOINT
    0x81,                           // bEndpointAddress = IN endpoint 1
    0x01,                           // bmAttributes = Isochronous, asynchronous
    0x60, 0x00,                     // wMaxPacketSize = 96 bytes (48kHz x 2 bytes x 1 ch / 1ms)
    0x01,                           // bInterval = 1 (1 ms)
    0x00,                           // bRefresh
    0x00,                           // bSynchAddress

    /* Class-Specific Isochronous Audio Data Endpoint Descriptor */
    0x07,                           // bLength
    0x25,                           // bDescriptorType = CS_ENDPOINT
    0x01,                           // bDescriptorSubtype = EP_GENERAL
    0x00,                           // bmAttributes
    0x00,                           // bLockDelayUnits
    0x00, 0x00                      // wLockDelay
};


/* High_Speed USB Standard Device Descriptor */
__ALIGN_BEGIN static uint8_t USBD_AUDIO_DeviceQualifierDesc[USB_LEN_DEV_QUALIFIER_DESC] __ALIGN_END =
{
  USB_LEN_DEV_QUALIFIER_DESC,
  USB_DESC_TYPE_DEVICE_QUALIFIER,
  0x00,
  0x02,
  0x00,
  0x00,
  0x00,
  0x40,
  0x01,
  0x00,
};


/**
  * @brief  USBD_AUDIO_Init
  *         Initialize the AUDIO interface
  * @param  pdev: device instance
  * @param  cfgidx: Configuration index
  * @retval status
  */
static uint8_t  USBD_AUDIO_Init(USBD_HandleTypeDef *pdev, uint8_t cfgidx)
{
  USBD_AUDIO_HandleTypeDef   *haudio;

  /* Open EP IN for MIC */
  USBD_LL_OpenEP(pdev, AUDIO_IN_EP, USBD_EP_TYPE_ISOC, AUDIO_IN_PACKET);
  pdev->ep_in[AUDIO_IN_EP & 0xFU].is_used = 1U;

  /* Allocate Audio structure */
  pdev->pClassData = USBD_malloc(sizeof(USBD_AUDIO_HandleTypeDef));
  if (pdev->pClassData == NULL)
  {
    return USBD_FAIL;
  }
  /* Binding Class_func */
  haudio = (USBD_AUDIO_HandleTypeDef *) pdev->pClassData;
  haudio->alt_setting = 0U;
  haudio->offset = AUDIO_OFFSET_UNKNOWN;
  haudio->wr_ptr = 0U;
  haudio->rd_ptr = 0U;
  haudio->rd_enable = 1U;
	
  /* Initialize the Audio input (MIC) Hardware layer */
  if (((USBD_AUDIO_ItfTypeDef *)pdev->pUserData[0])->Init(AUDIO_IN_FREQ,		/* Actually is AUDIO_Init_FS() */
                                                       24U,   /* 24bit */
                                                       1U) != 0)
  {
    return USBD_FAIL;
  }

  /* Prepare IN endpoint for first packet */
  USBD_LL_Transmit(pdev, AUDIO_IN_EP, haudio->buffer, AUDIO_IN_PACKET);

  return USBD_OK;
}


/**
  * @brief  USBD_AUDIO_Init
  *         DeInitialize the AUDIO layer
  * @param  pdev: device instance
  * @param  cfgidx: Configuration index
  * @retval status
  */
static uint8_t  USBD_AUDIO_DeInit(USBD_HandleTypeDef *pdev, uint8_t cfgidx)
{
  /* Close EP IN */
  USBD_LL_CloseEP(pdev, AUDIO_IN_EP);
  pdev->ep_in[AUDIO_IN_EP & 0xFU].is_used = 0U;

  if (pdev->pClassData != NULL)
  {
    ((USBD_AUDIO_ItfTypeDef *)pdev->pUserData[0])->DeInit(0U);
    USBD_free(pdev->pClassData);
    pdev->pClassData = NULL;
  }

  return USBD_OK;
}


/**
  * @brief  USBD_AUDIO_Setup
  *         Handle the AUDIO specific requests
  * @param  pdev: instance
  * @param  req: usb requests
  * @retval status
  */
static uint8_t USBD_AUDIO_Setup(USBD_HandleTypeDef *pdev,
                                USBD_SetupReqTypedef *req)
{
    USBD_AUDIO_HandleTypeDef *haudio;
    uint16_t len;
    uint8_t *pbuf;
    uint16_t status_info = 0U;
    uint8_t ret = USBD_OK;

    haudio = (USBD_AUDIO_HandleTypeDef *)pdev->pClassData;

    switch (req->bmRequest & USB_REQ_TYPE_MASK)
    {
    case USB_REQ_TYPE_CLASS:
        switch (req->bRequest)
        {
        case AUDIO_REQ_GET_CUR:
            AUDIO_REQ_GetCurrent(pdev, req);
            break;

        case AUDIO_REQ_SET_CUR:
            AUDIO_REQ_SetCurrent(pdev, req);
            break;

        case AUDIO_REQ_GET_MIN:
        {
            int16_t vol_min = 0x0000;
            USBD_CtlSendData(pdev, (uint8_t *)&vol_min, sizeof(vol_min));
            break;
        }

        case AUDIO_REQ_GET_MAX:
        {
            int16_t vol_max = 0x6400;
            USBD_CtlSendData(pdev, (uint8_t *)&vol_max, sizeof(vol_max));
            break;
        }

        case AUDIO_REQ_GET_RES:
        {
            int16_t vol_res = 0x0100;
            USBD_CtlSendData(pdev, (uint8_t *)&vol_res, sizeof(vol_res));
            break;
        }

        default:
            USBD_CtlError(pdev, req);
            ret = USBD_FAIL;
            break;
        }
        break;

    case USB_REQ_TYPE_STANDARD:
        switch (req->bRequest)
        {
        case USB_REQ_GET_STATUS:
            if (pdev->dev_state == USBD_STATE_CONFIGURED)
            {
                USBD_CtlSendData(pdev, (uint8_t *)(void *)&status_info, 2U);
            }
            else
            {
                USBD_CtlError(pdev, req);
                ret = USBD_FAIL;
            }
            break;

        case USB_REQ_GET_DESCRIPTOR:
            if ((req->wValue >> 8) == AUDIO_DESCRIPTOR_TYPE)
            {
                pbuf = USBD_AUDIO_CfgDesc + 18;
                len = MIN(USB_AUDIO_DESC_SIZ, req->wLength);
                USBD_CtlSendData(pdev, pbuf, len);
            }
            break;

        case USB_REQ_GET_INTERFACE:
            if (pdev->dev_state == USBD_STATE_CONFIGURED)
            {
                USBD_CtlSendData(pdev, (uint8_t *)(void *)&haudio->alt_setting, 1U);
            }
            else
            {
                USBD_CtlError(pdev, req);
                ret = USBD_FAIL;
            }
            break;

        case USB_REQ_SET_INTERFACE:
            if (pdev->dev_state == USBD_STATE_CONFIGURED)
            {
                uint8_t alt = (uint8_t)(req->wValue);

                haudio->alt_setting = alt;

                if (alt == 1)
                {
                    if(USBD_LL_OpenEP(pdev,
                                   AUDIO_IN_EP,
                                   USBD_EP_TYPE_ISOC,
                                   AUDIO_IN_PACKET) == USBD_OK)
					{
						uart2_print("OpenEP Successful\r\n");
						
					}else
					{
						uart2_print("OpenEP Filed\r\n");
					}
                    pdev->ep_in[AUDIO_IN_EP & 0xFU].is_used = 1U;

                    if(USBD_LL_Transmit(pdev,
                                     AUDIO_IN_EP,
                                     (uint8_t *)i2s_dma_buf,
                                     AUDIO_IN_PACKET) == USBD_OK)
					{
						uart2_print("Transmit Successful\r\n");
						
					}else{
						uart2_print("Transmit Filed\r\n");
					}
                }
                else if (alt == 0)
                {
                    USBD_LL_CloseEP(pdev, AUDIO_IN_EP);
                    pdev->ep_in[AUDIO_IN_EP & 0xFU].is_used = 0U;
                }
            }
            else
            {
                USBD_CtlError(pdev, req);
                ret = USBD_FAIL;
            }
            break;

        default:
            USBD_CtlError(pdev, req);
            ret = USBD_FAIL;
            break;
        }
        break;

    default:
        USBD_CtlError(pdev, req);
        ret = USBD_FAIL;
        break;
    }

    return ret;
}



/**
  * @brief  USBD_AUDIO_GetCfgDesc
  *         return configuration descriptor
  * @param  speed : current device speed
  * @param  length : pointer data length
  * @retval pointer to descriptor buffer
  */
static uint8_t  *USBD_AUDIO_GetCfgDesc(uint16_t *length)
{
  *length = sizeof(USBD_AUDIO_CfgDesc);

  return USBD_AUDIO_CfgDesc;
}


__ALIGN_BEGIN static uint8_t usb_buf[96] __ALIGN_END;
static uint32_t sample_index = 0;

static void FillAudioFrame(void)
{
    float freq = 1000.0f; // 测试频率 1kHz
    float amp  = 10000.0f;

    for (int i = 0; i < 48; i++) {
        int16_t sample = (int16_t)(amp * sinf(2.0f * 3.1415926f * freq * sample_index / 48000.0f));
        usb_buf[i*2]   = sample & 0xFF;
        usb_buf[i*2+1] = (sample >> 8) & 0xFF;

        sample_index++;
        if (sample_index >= 48000) sample_index = 0;
    }
}
/**
  * @brief  USBD_AUDIO_DataIn
  *         handle data IN Stage
  * @param  pdev: device instance
  * @param  epnum: endpoint index
  * @retval status
  */
static uint8_t  USBD_AUDIO_DataIn(USBD_HandleTypeDef *pdev, uint8_t epnum)
{
	uart2_print("Data_In\r\n");
  USBD_AUDIO_HandleTypeDef *haudio = (USBD_AUDIO_HandleTypeDef *)pdev->pClassData;
	FillAudioFrame();
  HAL_PCD_EP_Flush(pdev->pData, 0x81);
      // 调用 LL 发送 USB 音频帧（ISO IN）
  HAL_PCD_EP_Transmit(pdev->pData, AUDIO_IN_EP, usb_buf, 96);
  return USBD_OK;
}


/**
  * @brief  USBD_AUDIO_EP0_RxReady
  *         handle EP0 Rx Ready event
  * @param  pdev: device instance
  * @retval status
  */
static uint8_t  USBD_AUDIO_EP0_RxReady(USBD_HandleTypeDef *pdev)
{
  USBD_AUDIO_HandleTypeDef   *haudio;
  haudio = (USBD_AUDIO_HandleTypeDef *) pdev->pClassData;

if (haudio->control.cmd == AUDIO_REQ_SET_CUR)
{
    if (haudio->control.unit == 0x02) // Feature Unit ID
    {
        ((USBD_AUDIO_ItfTypeDef *)pdev->pUserData[0])->MuteCtl(haudio->control.data[0]);
        haudio->control.cmd = 0U;
        haudio->control.len = 0U;
    }
}

  return USBD_OK;
}
/**
  * @brief  USBD_AUDIO_EP0_TxReady
  *         handle EP0 TRx Ready event
  * @param  pdev: device instance
  * @retval status
  */
static uint8_t  USBD_AUDIO_EP0_TxReady(USBD_HandleTypeDef *pdev)
{
  /* Only OUT control data are processed */
  return USBD_OK;
}
/**
  * @brief  USBD_AUDIO_SOF
  *         handle SOF event
  * @param  pdev: device instance
  * @retval status
  */
uint8_t USBD_AUDIO_SOF(USBD_HandleTypeDef *pdev)
{
	uart2_print("SOF_In\r\n");
    return USBD_OK;
}

/**
  * @brief  USBD_AUDIO_SOF
  *         handle SOF event
  * @param  pdev: device instance
  * @retval status
  */
void  USBD_AUDIO_Sync(USBD_HandleTypeDef *pdev, AUDIO_OffsetTypeDef offset)
{
	uart2_print("AUDIO_SYNC_In\r\n");
}


/**
  * @brief  USBD_AUDIO_IsoINIncomplete
  *         handle data ISO IN Incomplete event
  * @param  pdev: device instance
  * @param  epnum: endpoint index
  * @retval status
  */
static uint8_t  USBD_AUDIO_IsoINIncomplete(USBD_HandleTypeDef *pdev, uint8_t epnum)
{
	uart2_print("ISO_IN_In\r\n");
	FillAudioFrame(); // 填充 usb_buf
    USBD_LL_Transmit(pdev, AUDIO_IN_EP, usb_buf, AUDIO_IN_PACKET);
  return USBD_OK;
}
/**
  * @brief  USBD_AUDIO_IsoOutIncomplete
  *         handle data ISO OUT Incomplete event
  * @param  pdev: device instance
  * @param  epnum: endpoint index
  * @retval status
  */
static uint8_t  USBD_AUDIO_IsoOutIncomplete(USBD_HandleTypeDef *pdev, uint8_t epnum)
{
	uart2_print("ISO_OUT_In\r\n");
  return USBD_OK;
}
/**
  * @brief  USBD_AUDIO_DataOut
  *         handle data OUT Stage
  * @param  pdev: device instance
  * @param  epnum: endpoint index
  * @retval status
  */
static uint8_t  USBD_AUDIO_DataOut(USBD_HandleTypeDef *pdev, uint8_t epnum)
{
  /* MIC ����Ҫ OUT �˵� */
  return USBD_OK;
}


/**
  * @brief  AUDIO_Req_GetCurrent
  *         Handles the GET_CUR Audio control request.
  * @param  pdev: instance
  * @param  req: setup class request
  * @retval status
  */
static void AUDIO_REQ_GetCurrent(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req)
{
    uint8_t response[2] = {0x00, 0x00}; 

    uint8_t control_selector = (req->wValue >> 8) & 0xFF; 

    if (control_selector == 0x01) // MUTE_CONTROL
    {
        response[0] = 0x00; 
        USBD_CtlSendData(pdev, response, req->wLength); // ע�ⳤ���� wLength
    }
    else if(control_selector == 0x02)
    {
        response[0] = 0x32; // ����50%
        USBD_CtlSendData(pdev, response, req->wLength); // ע�ⳤ���� wLength
    }else{
				USBD_CtlError(pdev, req); // ��֧�ֵĿ��Ʒ��� STALL
		}
}


/**
  * @brief  AUDIO_Req_SetCurrent
  *         Handles the SET_CUR Audio control request.
  * @param  pdev: instance
  * @param  req: setup class request
  * @retval status
  */
static void AUDIO_REQ_SetCurrent(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req)
{
  USBD_AUDIO_HandleTypeDef   *haudio;
  haudio = (USBD_AUDIO_HandleTypeDef *) pdev->pClassData;

  if (req->wLength)
  {
    /* Prepare the reception of the buffer over EP0 */
    USBD_CtlPrepareRx(pdev, haudio->control.data, req->wLength);

    haudio->control.cmd = AUDIO_REQ_SET_CUR;     /* Set the request value */
    haudio->control.len = (uint8_t)req->wLength; /* Set the request data length */
    haudio->control.unit = HIBYTE(req->wIndex);  /* Set the request target unit */
  }
}


/**
* @brief  USBD_AUDIO_RegisterInterface
* @param  fops: Audio interface callback
* @retval status
*/
uint8_t  USBD_AUDIO_RegisterInterface(USBD_HandleTypeDef *pdev,
                                      USBD_AUDIO_ItfTypeDef *fops)
{
  if (fops != NULL)
  {
    pdev->pUserData[0] = fops;
  }

  return USBD_OK;
}

/**
* @brief  DeviceQualifierDescriptor
*         return Device Qualifier descriptor
* @param  length : pointer data length
* @retval pointer to descriptor buffer
*/
static uint8_t  *USBD_AUDIO_GetDeviceQualifierDesc(uint16_t *length)
{
  *length = sizeof(USBD_AUDIO_DeviceQualifierDesc);

  return USBD_AUDIO_DeviceQualifierDesc;
}


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
