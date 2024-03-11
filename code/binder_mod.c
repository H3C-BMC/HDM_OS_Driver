/*****************************************************************************
                版权所有(C)，2007-2022，杭州华三通信技术有限公司
------------------------------------------------------------------------------
                            binder_mod.c
  产 品 名: VERSION
  模 块 名:
  生成日期: 2022年3月22日
  作    者: x22827
  文件描述: usb binder driver

------------------------------------------------------------------------------
   修改历史
   日期        姓名             描述
  --------------------------------------------------------------------------

*****************************************************************************/

#ifdef __cplusplus
extern "C"{
#endif /* __cplusplus */


#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/uaccess.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/workqueue.h>
#include <linux/usb.h>
#include <linux/usb/input.h>
#include <linux/hid.h>
#include <linux/version.h>
#include <linux/utsname.h>
#include "type.h"
#include "drv_error.h"
#include "pipe.h"

#include "binder_mod.h"

#define CMD_HIGH_BITS(cmd)      ((cmd >> 8) & 0xFF)
#define CMD_LOW_BITS(cmd)       (cmd & 0xFF)
#define ID_HIGH_BITS(id)        ((id >> 8) & 0xFF)
#define ID_LOW_BITS(id)         (id & 0xFF)

/*global var*/
BINDER_MOD_S g_stBinder;
STATIC struct class *g_pstBinderClass = NULL;
STATIC struct cdev g_stBinderCdev;
STATIC dev_t g_ulDevId;
STATIC INT g_iPipeOut = -1;
STATIC INT g_iPipeIn = -1;
struct usb_device *g_pstUsbDev;
STATIC struct workqueue_struct *g_pstQueue = NULL;
STATIC BINDER_WORK_S g_stWorkOut;
STATIC BINDER_WORK_S g_stWorkIn;
STATIC USHORT g_usSessionId = 0;
STATIC spinlock_t  g_stUrbInLock;
STATIC spinlock_t  g_stUrbOutLock;

INT g_iPktHeadLen = USB_BINDER_PKT_HEAD_LEN;

STATIC INT binder_send_osinfo(VOID);
STATIC INT binder_clear_osinfo(VOID);

/*func def*/
VOID print_handle_urb_error(struct urb *pstUrb)
{
    if (NULL == pstUrb)
    {
        return;
    }
    if (-EPIPE == pstUrb->status)
    {
        /*try to recover endpoint*/
        if (pstUrb == g_stBinder.pstUrbOut)
        {
            usb_clear_halt(g_pstUsbDev, g_iPipeOut);
            printk("out endpoint is now stalled.\n");
        }
        else if (pstUrb == g_stBinder.pstUrbIn)
        {
            usb_clear_halt(g_pstUsbDev, g_iPipeIn);
            printk("in endpoint is now stalled.\n");
        }
    }
}

INT init_resource(VOID)
{
    INT iRetval = 0;

#if LINUX_VERSION_CODE>= KERNEL_VERSION(2,6,36)
    /*......codes under version newer than 2.6.36......*/
    if (NULL == g_stBinder.pcUsbBufVirtOut)
    {
        g_stBinder.pcUsbBufVirtOut = usb_alloc_coherent(g_pstUsbDev, g_stBinder.uiUsbBufLen,
                                                        GFP_ATOMIC, &g_stBinder.ulUsbBufOutPhysAddr);
        if (!g_stBinder.pcUsbBufVirtOut)
        {
            iRetval = -ENOMEM;
            goto out_alloc_error;
        }
    }

    if (NULL == g_stBinder.pcUsbBufVirtIn)
    {
        g_stBinder.pcUsbBufVirtIn = usb_alloc_coherent(g_pstUsbDev, g_stBinder.uiUsbBufLen,
                                                       GFP_ATOMIC, &g_stBinder.ulUsbBufInPhysAddr);
        if (!g_stBinder.pcUsbBufVirtIn)
        {
            iRetval = -ENOMEM;
            goto in_alloc_error;
        }
    }
#else
     /*......codes under version older than 2.6.36......*/
    if (NULL == g_stBinder.pcUsbBufVirtOut)
    {
        g_stBinder.pcUsbBufVirtOut = usb_buffer_alloc(g_pstUsbDev, g_stBinder.uiUsbBufLen,
                                                      GFP_ATOMIC, &g_stBinder.ulUsbBufOutPhysAddr);
        if (!g_stBinder.pcUsbBufVirtOut)
        {
            iRetval = -ENOMEM;
            goto out_alloc_error;
        }
    }
    if (NULL == g_stBinder.pcUsbBufVirtIn)
    {
        g_stBinder.pcUsbBufVirtIn = usb_buffer_alloc(g_pstUsbDev, g_stBinder.uiUsbBufLen,
                                                     GFP_ATOMIC, &g_stBinder.ulUsbBufInPhysAddr);
        if (!g_stBinder.pcUsbBufVirtIn)
        {
            iRetval = -ENOMEM;
            goto in_alloc_error;
        }
    }
#endif
    if (NULL == g_stBinder.pstUrbIn)
    {
        g_stBinder.pstUrbIn = usb_alloc_urb(0, GFP_KERNEL);
        if (!g_stBinder.pstUrbIn)
        {
            iRetval = -ENOMEM;
            goto urbin_alloc_error;
        }
    }
    if (NULL == g_stBinder.pstUrbOut)
    {
        g_stBinder.pstUrbOut = usb_alloc_urb(0, GFP_KERNEL);
        if (!g_stBinder.pstUrbOut)
        {
            iRetval = -ENOMEM;
            goto urbout_alloc_error;
        }
    }

    return BINDER_SUCCESS;

urbout_alloc_error:
    usb_kill_urb(g_stBinder.pstUrbIn);
    usb_free_urb(g_stBinder.pstUrbIn);
    g_stBinder.pstUrbIn = NULL;
urbin_alloc_error:
#if LINUX_VERSION_CODE>= KERNEL_VERSION(2,6,36)
    usb_free_coherent(g_pstUsbDev, g_stBinder.uiUsbBufLen, g_stBinder.pcUsbBufVirtIn,
                      g_stBinder.ulUsbBufInPhysAddr);
#else
    usb_buffer_free(g_pstUsbDev, g_stBinder.uiUsbBufLen, g_stBinder.pcUsbBufVirtIn,
                    g_stBinder.ulUsbBufInPhysAddr);
#endif
    g_stBinder.pcUsbBufVirtIn = NULL;
in_alloc_error:
#if LINUX_VERSION_CODE>= KERNEL_VERSION(2,6,36)
    usb_free_coherent(g_pstUsbDev, g_stBinder.uiUsbBufLen, g_stBinder.pcUsbBufVirtOut,
                      g_stBinder.ulUsbBufOutPhysAddr);
#else
    usb_buffer_free(g_pstUsbDev, g_stBinder.uiUsbBufLen, g_stBinder.pcUsbBufVirtOut,
                    g_stBinder.ulUsbBufOutPhysAddr);
#endif
    g_stBinder.pcUsbBufVirtOut = NULL;
out_alloc_error:
    return iRetval;
}


VOID free_usb_resource(VOID)
{
    if (g_stBinder.pstUrbOut)
    {
        usb_kill_urb(g_stBinder.pstUrbOut);
        usb_free_urb(g_stBinder.pstUrbOut);
        g_stBinder.pstUrbOut = NULL;
    }

    if (g_stBinder.pstUrbIn)
    {
        usb_kill_urb(g_stBinder.pstUrbIn);
        usb_free_urb(g_stBinder.pstUrbIn);
        g_stBinder.pstUrbIn = NULL;
    }

#if LINUX_VERSION_CODE>= KERNEL_VERSION(2,6,36)
    /*......codes under version newer than 2.6.36......*/
    if (g_stBinder.pcUsbBufVirtOut)
    {
        usb_free_coherent(g_pstUsbDev, g_stBinder.uiUsbBufLen, g_stBinder.pcUsbBufVirtOut,
                          g_stBinder.ulUsbBufOutPhysAddr);
        g_stBinder.pcUsbBufVirtOut = NULL;
    }
    if (g_stBinder.pcUsbBufVirtIn)
    {
        usb_free_coherent(g_pstUsbDev, g_stBinder.uiUsbBufLen, g_stBinder.pcUsbBufVirtIn,
                          g_stBinder.ulUsbBufInPhysAddr);
        g_stBinder.pcUsbBufVirtIn = NULL;
    }

#else
     /*......codes under version older than 2.6.36......*/
    if (g_stBinder.pcUsbBufVirtOut)
    {
        usb_buffer_free(g_pstUsbDev, g_stBinder.uiUsbBufLen, g_stBinder.pcUsbBufVirtOut,
                        g_stBinder.ulUsbBufOutPhysAddr);
        g_stBinder.pcUsbBufVirtOut = NULL;
    }
    if (g_stBinder.pcUsbBufVirtIn)
    {
        usb_buffer_free(g_pstUsbDev, g_stBinder.uiUsbBufLen, g_stBinder.pcUsbBufVirtIn,
                        g_stBinder.ulUsbBufInPhysAddr);
        g_stBinder.pcUsbBufVirtIn = NULL;
    }
#endif
}
/*context: not interrupt*/
STATIC VOID work_handle(struct work_struct *pstWork)
{
    INT i = 0;
    BINDER_WORK_S *pstBinderWork = (BINDER_WORK_S *)pstWork;

    if (!(IsUsbDetected()))
    {
        return;
    }

    /*process out work*/
    if (DIR_OUT == pstBinderWork->iDirChl)
    {
        if (g_stBinder.pstUrbOut->status)
        {
            PipeSendDataAfterRun();
            print_handle_urb_error(g_stBinder.pstUrbOut);
            return;
        }
        /*data all out*/
        if (IsPipeSendComplete())
        {
            PipeSendDataAfterRun();
            return;
        }

        /*continue sned data*/
        PipeTxHandler();
    }
    /*process in work*/
    else if (DIR_IN == pstBinderWork->iDirChl)
    {
        if (-EPIPE == g_stBinder.pstUrbIn->status)
        {
            print_handle_urb_error(g_stBinder.pstUrbIn);
        }
        
        if (NULL == g_stBinder.pstUrbIn)
        {
            return;
        }
        /* print in data*/
        DEBUG_PRINT("Recv data len : 0x%x\n",g_stBinder.pstUrbIn->actual_length);
        DEBUG_PRINT("usb_binder urb in data : \n");
        for (i = 0; i < g_stBinder.pstUrbIn->actual_length; i++)
        {
            DEBUG_PRINT("0x%x ", g_stBinder.pcUsbBufVirtIn[i]);
        }
        DEBUG_PRINT("\n");

        PipeRxHandler(g_stBinder.pcUsbBufVirtIn, g_stBinder.pstUrbIn->actual_length, g_stBinder.pstUrbIn->status);

        /* submit urb again*/
        spin_lock(&g_stUrbInLock);
        if (usb_submit_urb(g_stBinder.pstUrbIn, GFP_ATOMIC))
        {
            printk("work_handle : usb_submit_urb error\n");
        }
        spin_unlock(&g_stUrbInLock);
    }

    return;
}

/*context:  interrupt*/
STATIC VOID binder_in_complete(struct urb *pstUrb)
{
    switch (pstUrb->status)
    {
        case 0:         /* success */
        case -EPIPE:    /*should clear the halt */
            break;
        case -ECONNRESET:   /* unlink */
        case -ENOENT:
        case -ESHUTDOWN:
        default:        /* error */
        {
            printk("binder_in_complete : failed,errcode:%d\n", pstUrb->status);
        }
    }
    /*submit data_in work*/
    INIT_WORK((struct work_struct *)&g_stWorkIn, work_handle);
    g_stWorkIn.iDirChl = DIR_IN;
    queue_work(g_pstQueue, (struct work_struct *)&g_stWorkIn);

    return;
}

/*context:  interrupt*/
STATIC VOID binder_out_complete(struct urb *pstUrb)
{
    switch (pstUrb->status)
    {
        case 0:     /* success */
        case -EPIPE:    /*should clear the halt */
            break;
        case -ECONNRESET:   /* unlink */
        case -ENOENT:
        case -ESHUTDOWN:
        default:        /* error */
        {
            printk("binder_out_complete : failed,errcode:%d\n", pstUrb->status);
            PipeSendDataAfterRun();
            return;
        }
    }
    /*submit data_out work*/
    INIT_WORK((struct work_struct *)&g_stWorkOut, work_handle);
    g_stWorkOut.iDirChl = DIR_OUT;
    queue_work(g_pstQueue, (struct work_struct *)&g_stWorkOut);

    return;
}

/*send data by urb*/
INT usb_send_urb(UCHAR * pucData, UINT uiLen)
{
    if ((!pucData) || (!uiLen) || (!g_stBinder.pstUrbOut))
    {
        return BINDER_PARAMETER_INVLID;
    }
    spin_lock(&g_stUrbOutLock);
    memcpy(g_stBinder.pcUsbBufVirtOut, pucData, uiLen);

    usb_fill_bulk_urb(g_stBinder.pstUrbOut, g_pstUsbDev, g_iPipeOut, g_stBinder.pcUsbBufVirtOut, uiLen, binder_out_complete, NULL);
    g_stBinder.pstUrbOut->transfer_dma = g_stBinder.ulUsbBufOutPhysAddr;
    g_stBinder.pstUrbOut->transfer_flags |= URB_NO_TRANSFER_DMA_MAP;
    /* submit urb */
    if (usb_submit_urb(g_stBinder.pstUrbOut, GFP_ATOMIC))
    {
        printk("usb_send_urb: usb_submit_urb error\n");
        spin_unlock(&g_stUrbOutLock);
        return BINDER_USB_SEND_URB_ERROR;
    }
    spin_unlock(&g_stUrbOutLock);

    return BINDER_SUCCESS;
}

/* probe func */
STATIC INT usb_binder_probe(struct usb_interface *pstUsbIntf, const struct usb_device_id *pstDevId)
{
    struct usb_device *pstDev = interface_to_usbdev(pstUsbIntf);
    struct usb_host_interface *pstHostInterface = NULL;
    struct usb_endpoint_descriptor *pstEndpoint = NULL;
    struct usb_interface_descriptor *pstInterfaceDescriptor = NULL;
    struct usb_pipe_device stPipeDevice;
    INT i = 0;
    INT iEndpointAddress = 0;
    INT iRet = 0;

    DEBUG_PRINT("usb_binder_probe enter.\n");
    memset((VOID*)&g_stBinder,0,sizeof(BINDER_MOD_S));
    memset(&stPipeDevice, 0, sizeof(stPipeDevice));

    g_pstUsbDev = pstDev;
    /* get enpoint info for debug */
    pstHostInterface = pstUsbIntf->cur_altsetting;
    pstInterfaceDescriptor = &pstHostInterface->desc;

    for (i = 0; i < pstInterfaceDescriptor->bNumEndpoints; i++)
    {
        pstEndpoint = &pstHostInterface->endpoint[i].desc;
        iEndpointAddress = pstEndpoint->bEndpointAddress;
        DEBUG_PRINT("endpoint[%d] addr is %x.\n",i, iEndpointAddress);

        iEndpointAddress = iEndpointAddress &0x80;
        if (0 == iEndpointAddress)
        {
            g_iPipeOut = usb_sndbulkpipe(pstDev, pstEndpoint->bEndpointAddress);
            DEBUG_PRINT("endpoint[%d] is dir out.\n",i);
        }
        else
        {
            g_iPipeIn = usb_rcvbulkpipe(pstDev, pstEndpoint->bEndpointAddress);
            DEBUG_PRINT("endpoint[%d] is dir in.\n",i);
        }
    }

    if (NULL == pstEndpoint)
    {
        printk("Not found any endpoint.\n");
        return BINDER_USB_ENDPOINT_NULL;
    }

    g_stBinder.uiUsbBufLen = pstEndpoint->wMaxPacketSize;
    if (init_resource())
    {
        return  -ENOMEM;
    }

    spin_lock(&g_stUrbInLock);
    usb_fill_bulk_urb(g_stBinder.pstUrbIn, pstDev, g_iPipeIn, g_stBinder.pcUsbBufVirtIn,
                      g_stBinder.uiUsbBufLen, binder_in_complete, NULL);
    g_stBinder.pstUrbIn->transfer_dma = g_stBinder.ulUsbBufInPhysAddr;
    g_stBinder.pstUrbIn->transfer_flags |= URB_NO_TRANSFER_DMA_MAP;

    /*recv data from usb binder device.*/
    iRet = usb_submit_urb(g_stBinder.pstUrbIn, GFP_KERNEL);
    if (iRet)
    {
        printk("usb_binder_probe: usb_submit_urb error.\n");
        spin_unlock(&g_stUrbInLock);
        free_usb_resource();
        return iRet;
    }
    spin_unlock(&g_stUrbInLock);

    /*init pipe*/
    stPipeDevice.uiMaxCount = g_stBinder.uiUsbBufLen;
    stPipeDevice.UsbSendData = usb_send_urb;
    init_pipe(&stPipeDevice);

    binder_send_osinfo();

    printk("usb_binder_probe: success.\n");
    return BINDER_SUCCESS;
}


/* usb disconnect func*/
STATIC VOID usb_binder_disconnect(struct usb_interface *pstUsbIntf)
{
    binder_clear_osinfo();
    exit_pipe();

    flush_workqueue(g_pstQueue);
    printk("usb_binder_disconnect.\n");
    return;
}


/*id_table*/
STATIC struct usb_device_id usb_binder_id_table [] = {
    { USB_DEVICE(0xFF58, 0x9527) }, /* only for debugging */
    { USB_DEVICE(0x1472, 0x3013) },
    { } /* Terminating entry */
};

/*usb_driver*/
STATIC struct usb_driver usb_binder_driver = {
    .name       = "binder",
    .probe      = usb_binder_probe,
    .disconnect = usb_binder_disconnect,
    .id_table   = usb_binder_id_table,
};

STATIC INT binder_ioctl(struct inode *pstInode, struct file *pstFile, UINT uiCmd, ULONG ulUsrArg)
{
    BINDER_PKT_S stPkt;
    BINDER_PRIVATE_DATA_S *pstBinder = (BINDER_PRIVATE_DATA_S *)pstFile->private_data;
    char acHead[USB_BINDER_PKT_HEAD_LEN];
    INT iRet = BINDER_SUCCESS;

    memset(&stPkt, 0, sizeof(stPkt));
    memset(acHead, 0, sizeof(acHead));

    if (!IsUsbDetected())
    {
        printk("Not found usb device.\n");
        return BINDER_PIPE_NOT_FOUND_USB_DEVICE;
    }

    switch (uiCmd)
    {
        case IOCTL_BINDER_INIT:
        {
            (VOID)ClearPipeDataQueue(pstBinder->pstRecvQue);
            g_iPktHeadLen = USB_BINDER_PKT_HEAD_LEN;
            acHead[0] = pstBinder->usSessionID & 0xff;          /* 低8位 */
            acHead[1] = (pstBinder->usSessionID >> 8) & 0xff;   /* 高8位 */
            acHead[2] = USB_BINDER_CTRL_CMD_CONNETCT;
            stPkt.pcData = acHead;
            stPkt.uiDataLen = g_iPktHeadLen;
            stPkt.uiTimeout = 0;
            if (stPkt.uiDataLen != PipeWriteByKernel(0, stPkt.pcData, stPkt.uiDataLen, stPkt.uiTimeout))
            {
                printk("binder_ioctl: init binder failed.\n");
                iRet = BINDER_IO_INIT_ERROR;
            }
            break;
        }
        case IOCTL_BINDER_READ:
        {
            if (copy_from_user(&stPkt, (VOID*)ulUsrArg, sizeof(BINDER_PKT_S)))
            {
                iRet = -EFAULT;
                break;
            }
            if (0 >= PipeRead(pstBinder->usSessionID, stPkt.pcData, stPkt.uiDataLen, stPkt.uiTimeout))
            {
				iRet = BINDER_IO_READ_ERROR;
            }
            break;
        }
        case IOCTL_BINDER_WRITE:
        {
            if (copy_from_user(&stPkt, (VOID*)ulUsrArg, sizeof(BINDER_PKT_S)))
            {
                iRet = -EFAULT;
                break;
            }

            if (stPkt.uiDataLen != PipeWrite(pstBinder->usSessionID, stPkt.pcData, stPkt.uiDataLen, stPkt.uiTimeout))
            {                
                iRet = BINDER_IO_WRITE_ERROR;
            }
            break;
        }
        case IOCTL_BINDER_EXIT:
        {
            break;
        }
        default:
        {
            printk("Invalid cmd 0x%x.\n", uiCmd);
            iRet = BINDER_IO_CMD_ERROR;
            break;
        }
    }


    return iRet;
}

STATIC LONG binder_ioctl_unlock(struct file *pstFile, UINT uiCmd, ULONG ulUsrArg)
{
    return (LONG)binder_ioctl(NULL, pstFile, uiCmd, ulUsrArg);
}

STATIC INT binder_open(struct inode *pstInode, struct file *pstFile)
{
    BINDER_PRIVATE_DATA_S *pstBinder = NULL;

    pstBinder = (BINDER_PRIVATE_DATA_S *)kmalloc(sizeof(BINDER_PRIVATE_DATA_S), GFP_ATOMIC);
    if (NULL == pstBinder)
    {
        return -ENOMEM;
    }
    memset(pstBinder, 0, sizeof(BINDER_PRIVATE_DATA_S));

    /* pipe buf for recv data */
    pstBinder->pstRecvQue = Pipe_malloc();
    if (NULL == pstBinder->pstRecvQue)
    {
        kfree(pstBinder);
        return -ENOMEM;
    }
    if (0 == (++g_usSessionId))
    {
        g_usSessionId++;
    }
    pstBinder->usSessionID = g_usSessionId;
    pstBinder->pstRecvQue->usId = pstBinder->usSessionID;
    pstFile->private_data = pstBinder;
    return 0;
}

STATIC INT binder_release(struct inode *pstInode, struct file *pstFile)
{
    BINDER_PRIVATE_DATA_S *pstBinder = pstFile->private_data;
    BINDER_PKT_S stPkt;
    CHAR acHead[USB_BINDER_PKT_HEAD_LEN];

    if (IsUsbDetected())
    {
        memset(&stPkt, 0, sizeof(stPkt));
        memset(acHead, 0, sizeof(acHead));
        acHead[0] = pstBinder->usSessionID & 0xff;
        acHead[1] = (pstBinder->usSessionID >> 8) & 0xff;
        acHead[2] = USB_BINDER_CTRL_CMD_DISCONNECT;
        stPkt.pcData = acHead;
        stPkt.uiDataLen = g_iPktHeadLen;
        stPkt.uiTimeout = 0;
        if (stPkt.uiDataLen != PipeWriteByKernel(0, stPkt.pcData, stPkt.uiDataLen, stPkt.uiTimeout))
        {
            printk("binder_release: send disconnect cmd failed\n");
        }
    }

    Pipe_free(pstBinder->pstRecvQue);
    kfree(pstBinder);
    pstFile->private_data = NULL;
    return 0;
}

struct file_operations binder_fops =
{
    .owner  =   THIS_MODULE,
    .open   =   binder_open,
    .release =  binder_release,
    .unlocked_ioctl  =     binder_ioctl_unlock,
};

STATIC INT binder_send_osinfo(VOID)
{
    BINDER_PKT_S stPkt;
    struct new_utsname stSysname;
    CHAR *pcBuf = NULL;

    memset(&stPkt, 0, sizeof(stPkt));
    memset(&stSysname, 0, sizeof(stSysname));

    memcpy(&stSysname, utsname(), sizeof(stSysname));

    /* 多申请一个包头 */
    pcBuf = (CHAR *)kmalloc(sizeof(stSysname) + USB_BINDER_PKT_HEAD_LEN, GFP_ATOMIC);
    if (NULL == pcBuf)
    {
        printk(KERN_ERR "binder: no memory enough.\n");
        return -ENOMEM;
    }
    /* 构造包头 */
    pcBuf[0] = ID_LOW_BITS(0);
    pcBuf[1] = ID_HIGH_BITS(0);
    pcBuf[2] = CMD_LOW_BITS(USB_BINDER_CTRL_CMD_SEND_OSINFO);
    pcBuf[3] = CMD_HIGH_BITS(USB_BINDER_CTRL_CMD_SEND_OSINFO);
    /* 数据跟在包头后面 */
    memcpy(&pcBuf[USB_BINDER_PKT_HEAD_LEN], &stSysname, sizeof(stSysname));

    stPkt.pcData = pcBuf;
    stPkt.uiDataLen = sizeof(stSysname) + USB_BINDER_PKT_HEAD_LEN;
    stPkt.uiTimeout = 0;
    if (stPkt.uiDataLen != PipeWriteByKernel(0, stPkt.pcData, stPkt.uiDataLen, stPkt.uiTimeout))
    {
        printk(KERN_ERR "binder_ioctl: PipeWrite len is not excepted len 0x%x\n", stPkt.uiDataLen);
        kfree(pcBuf);
        return BINDER_IO_INIT_ERROR;
    }

    kfree(pcBuf);
    return BINDER_SUCCESS;
}

STATIC INT binder_clear_osinfo(VOID)
{
    BINDER_PKT_S stPkt;
    CHAR acHead[USB_BINDER_PKT_HEAD_LEN];

    memset(&stPkt, 0, sizeof(stPkt));
    memset(acHead, 0, sizeof(acHead));

    acHead[0] = ID_LOW_BITS(0);
    acHead[1] = ID_HIGH_BITS(0);
    acHead[2] = CMD_LOW_BITS(USB_BINDER_CTRL_CMD_CLEAR_OSINFO);
    acHead[3] = CMD_HIGH_BITS(USB_BINDER_CTRL_CMD_CLEAR_OSINFO);
    stPkt.pcData = acHead;
    stPkt.uiDataLen = g_iPktHeadLen;
    stPkt.uiTimeout = 0;
    if (stPkt.uiDataLen != PipeWriteByKernel(0, stPkt.pcData, stPkt.uiDataLen, stPkt.uiTimeout))
    {
        printk("binder_clear_osinfo: clear os info failed\n");
        return BINDER_IO_INIT_ERROR;
    }
    return BINDER_SUCCESS;
}

STATIC INT binder_dev_init(VOID)
{
    INT iRetval = BINDER_SUCCESS;
    dev_t ulDevNo;
    struct device *pstDev = NULL;

    iRetval = alloc_chrdev_region(&ulDevNo, 0, 1, DEVICE_NAME);
    if (iRetval)
    {
        printk(KERN_ERR "binder_dev_init: unable to alloc_chrdev_region, %d.\n", iRetval);
        goto error;
    }

    cdev_init(&g_stBinderCdev, &binder_fops);
    g_stBinderCdev.owner = THIS_MODULE;
    g_stBinderCdev.ops = &binder_fops;

    iRetval = cdev_add(&g_stBinderCdev, ulDevNo, 1);
    if (iRetval)
    {
        iRetval = -ENODEV;
        printk(KERN_ERR "binder_dev_init: cdev_add error, %d.\n", iRetval);
        goto cdev_add_error;
    }

    g_pstBinderClass = class_create(THIS_MODULE, DEVICE_NAME);
    if (IS_ERR(g_pstBinderClass))
    {
        iRetval = PTR_ERR(g_pstBinderClass);
        printk(KERN_ERR "binder_dev_init: class_create failed\n");
        goto class_create_error;
    }
    pstDev = device_create(g_pstBinderClass, NULL, ulDevNo, NULL, DEVICE_NAME);
    if (IS_ERR(pstDev))
    {
        iRetval = PTR_ERR(pstDev);
        printk(KERN_ERR "binder_dev_init: device_create failed\n");
        goto device_create_error;
    }

    g_ulDevId = ulDevNo;
    return BINDER_SUCCESS;

    device_create_error:
    class_destroy(g_pstBinderClass);
    class_create_error:
    cdev_del(&g_stBinderCdev);
    cdev_add_error:
    unregister_chrdev_region(ulDevNo, 1);
    error:
    flush_workqueue(g_pstQueue);
    free_usb_resource();
    return iRetval;
}

STATIC INT __init binder_init(VOID)
{
    INT iRetval;

    printk("USB Device Host Driver(Version : 1.00.02) \n");

    memset(&g_stBinder, 0, sizeof(BINDER_MOD_S));

    g_pstQueue = create_singlethread_workqueue("binder_data_process");
    if (NULL == g_pstQueue)
    {
        printk(KERN_ERR "binder_init : create workqueue failed.\n");
        iRetval = BINDER_CREATE_WORKQUEUE_ERROR;
        goto error;
    }

    iRetval = usb_register(&usb_binder_driver);
    if (iRetval)
    {
        printk(KERN_ERR "usb_register failed, error = %d\n", iRetval);
        goto dev_init_error;
    }
    if (binder_dev_init())
    {
        iRetval = BINDER_DEVICE_INIT_ERROR;
        goto dev_init_error;
    }
    init_pipe_mod();

    spin_lock_init (&g_stUrbInLock);
    spin_lock_init (&g_stUrbOutLock);

    return BINDER_SUCCESS;

dev_init_error:
    usb_deregister(&usb_binder_driver);
error:
    return iRetval;
}


STATIC VOID __exit binder_exit(VOID)
{
    cdev_del(&g_stBinderCdev);
    flush_workqueue(g_pstQueue);
    usb_deregister(&usb_binder_driver);
    unregister_chrdev_region(g_ulDevId,1);
    device_destroy(g_pstBinderClass,g_ulDevId);
    class_destroy(g_pstBinderClass);
    destroy_workqueue(g_pstQueue);
    free_usb_resource();
    return ;
}


module_init(binder_init);
module_exit(binder_exit);


MODULE_AUTHOR("XUEFENG.");
MODULE_DESCRIPTION("BMC Binder Driver");
MODULE_LICENSE("GPL");

#ifdef __cplusplus
}
#endif /* __cplusplus */

