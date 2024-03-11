/*****************************************************************************
                版权所有(C)，2007-2022，杭州华三通信技术有限公司
------------------------------------------------------------------------------
                            pipe.c
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
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/uaccess.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/sched.h>
#include <linux/wait.h>
#include <linux/slab.h>
#include <linux/jiffies.h>
#include <linux/delay.h>
#include "type.h"
#include "pipe.h"
#include "drv_error.h"
#include "binder_mod.h"

STATIC struct usb_pipe_device g_stUsbPipe;
STATIC PIPE_QUEUE_T g_stPipeQueue;
STATIC INT g_iPipeInit = 0;
STATIC INT g_iSendDataFlag = 1;
STATIC INT g_iSendDataComplete = 0;
STATIC UINT g_uiTxDataLen = 0;
STATIC UINT g_uiTxDataPos = 0;
STATIC UINT g_uiTxZeroLenPkt = 0;
STATIC UCHAR *g_pucTxDataBuf = NULL;
STATIC INT g_iPipeInitFist = 1;

USHORT g_usPktSeq = 0;

extern INT g_iPktHeadLen;

PIPE_QUEUE_T *Pipe_find(INT iPipeId)
{
    PIPE_QUEUE_T *pstCurr;
    spin_lock(&g_stPipeQueue.stLock);
    list_for_each_entry(pstCurr, &g_stPipeQueue.stPipeList, stPipeList)
    {
        if (iPipeId == pstCurr->usId)
        {
            spin_unlock(&g_stPipeQueue.stLock);
            return pstCurr;
        }
    }
    spin_unlock(&g_stPipeQueue.stLock);
    return NULL;
}


PIPE_QUEUE_T *Pipe_malloc(VOID)
{
    PIPE_QUEUE_T *pstPipeQue = NULL;
    pstPipeQue = (PIPE_QUEUE_T *)kmalloc(sizeof(PIPE_QUEUE_T), GFP_ATOMIC);
    if (NULL == pstPipeQue)
    {
        printk("Init pipe failed, no enough memory.\n");
        return NULL;
    }
    memset(pstPipeQue, 0, sizeof(*pstPipeQue));
    spin_lock_init (&pstPipeQue->stLock);
    init_waitqueue_head(&pstPipeQue->stWait);
    init_waitqueue_head(&pstPipeQue->stWrWait);
    spin_lock(&g_stPipeQueue.stLock);
    list_add(&pstPipeQue->stPipeList, &g_stPipeQueue.stPipeList);
    spin_unlock(&g_stPipeQueue.stLock);
    return pstPipeQue;
}
VOID Pipe_free(PIPE_QUEUE_T *pstPipeQue)
{
    if (NULL != pstPipeQue)
    {
        spin_lock(&g_stPipeQueue.stLock);
        list_del(&pstPipeQue->stPipeList);
        spin_unlock(&g_stPipeQueue.stLock);

        kfree(pstPipeQue);
        pstPipeQue = NULL;
    }
    return;
}

VOID init_pipe(struct usb_pipe_device *pstDevice)
{
    if ((!g_iPipeInit) && (NULL != pstDevice))
    {
        g_stPipeQueue.usId = 0;
        g_stPipeQueue.iWrIndex = 0;
        g_stPipeQueue.iRdIndex = 0;
        g_stPipeQueue.iCount = 0;
        g_stPipeQueue.bAbnormal = BOOL_FALSE;
        spin_lock_init (&g_stPipeQueue.stLock);
        init_waitqueue_head(&g_stPipeQueue.stWait);
        init_waitqueue_head(&g_stPipeQueue.stWrWait);
        if (g_iPipeInitFist)
        {
            INIT_LIST_HEAD(&g_stPipeQueue.stPipeList);
            g_iPipeInitFist = 0;
        }
        g_iPipeInit = 1;

        memcpy(&g_stUsbPipe, pstDevice, sizeof(struct usb_pipe_device));
    }
    return ;
}
VOID init_pipe_mod(VOID)
{
    spin_lock_init (&g_stPipeQueue.stLock);
    init_waitqueue_head(&g_stPipeQueue.stWait);
    init_waitqueue_head(&g_stPipeQueue.stWrWait);
    if (g_iPipeInitFist)
    {
        INIT_LIST_HEAD(&g_stPipeQueue.stPipeList);
        g_iPipeInitFist = 0;
    }
}

VOID exit_pipe()
{
    g_iPipeInit=0;
    return ;
}

INT ClearPipeDataQueue (PIPE_QUEUE_T *pstPipeQue)
{
    if (NULL != pstPipeQue)
    {
        spin_lock(&pstPipeQue->stLock);
        memset(pstPipeQue->uiEntrySize, 0, sizeof(pstPipeQue->uiEntrySize));
        pstPipeQue->iRdIndex = 0;
        pstPipeQue->iWrIndex = 0;
        pstPipeQue->iCount = 0;
        pstPipeQue->bAbnormal = BOOL_FALSE;
        spin_unlock(&pstPipeQue->stLock);
    }
    return BINDER_SUCCESS;
}
VOID PipeSendDataAfterRun(VOID)
{
    if (g_pucTxDataBuf && g_iSendDataComplete)
    {
        kfree(g_pucTxDataBuf);
    }
    g_pucTxDataBuf = NULL;
    g_iSendDataFlag = 1;
    wake_up_interruptible(&g_stPipeQueue.stWrWait);
    return;
}

INT IsPipeSendComplete(VOID)
{
    return g_iSendDataComplete;
}

INT IsUsbDetected(VOID)
{
    return g_iPipeInit;
}

/*仿照usbcore HandTxHandler 发送策略*/
VOID PipeTxHandler (VOID)
{
    UINT uiLen = 0;
    UCHAR *pucBuffer = NULL;
    UCHAR *ucBufferHead = NULL;
    UINT uiMaxSize = 0;

    uiMaxSize = g_stUsbPipe.uiMaxCount;

    /*************************MAX PKT SIZE PACKET ***********************/
    if ((g_uiTxDataLen - g_uiTxDataPos) >= (uiMaxSize - g_iPktHeadLen))
    {
        uiLen     = uiMaxSize - g_iPktHeadLen;

        /* Buffer and Len to be transmitted */
        pucBuffer = g_pucTxDataBuf + g_iPktHeadLen + g_uiTxDataPos;
        ucBufferHead = g_pucTxDataBuf + g_uiTxDataPos;
        g_pucTxDataBuf[2] = uiLen & 0xFF;
        g_pucTxDataBuf[3] = (uiLen >> 8) & 0xFF;

        if ((g_pucTxDataBuf != ucBufferHead) && (0 != g_iPktHeadLen))
        {
            memcpy(ucBufferHead, g_pucTxDataBuf, g_iPktHeadLen);
        }

        /* Increment Buffer Pointer */
        g_uiTxDataPos += uiLen;

        /* Transferred All Bytes. Check if a Zero length packet to be Sent*/
        if (g_uiTxDataLen == g_uiTxDataPos)
        {
            g_iSendDataComplete = 1;      /* Set End of Transmission */
            DEBUG_PRINT("HandleTxData():EOT:Partial Pkt Len = (%d)\n", uiLen);
        }

        if (0 != g_stUsbPipe.UsbSendData(ucBufferHead, uiLen + g_iPktHeadLen))
        {
            g_iSendDataComplete = 1;
            PipeSendDataAfterRun();
        }
        return;
    }

    /*************************PARTIAL SIZE PACKET ***********************/
    /* If Remaing Length is < than MaxPacket Size , Send the remaining Bytes */
    if (g_uiTxDataLen != g_uiTxDataPos)
    {
        uiLen     = g_uiTxDataLen - g_uiTxDataPos;

        /* Buffer and Len to be transmitted */
        pucBuffer = g_pucTxDataBuf + g_iPktHeadLen + g_uiTxDataPos;
        ucBufferHead = g_pucTxDataBuf + g_uiTxDataPos;
        g_pucTxDataBuf[2] = uiLen & 0xFF;
        g_pucTxDataBuf[3] = (uiLen >> 8) & 0xFF;

        if ((g_pucTxDataBuf != ucBufferHead) && (0 != g_iPktHeadLen))
        {
            memcpy(ucBufferHead, g_pucTxDataBuf, g_iPktHeadLen);
        }


        /* Increment Buffer Pointer */
        g_uiTxDataPos += uiLen;

        /* EOT is reached because of partial length packet */
        g_iSendDataComplete = 1;	/* Set End of Transmission */

        if (0 != g_stUsbPipe.UsbSendData(ucBufferHead, uiLen + g_iPktHeadLen))
        {
            g_iSendDataComplete = 1;
            PipeSendDataAfterRun();
        }
        return;
    }

    /************************** ZERO LEN PACKET ***********************/
    /* If RemainingLength is zero, and Zero Len packet can be sent */
    if ((g_uiTxDataLen == g_uiTxDataPos) && (g_uiTxZeroLenPkt))
    {
        DEBUG_PRINT("HandleTxData():EOT:Sending Zero Len Packet for EP%d\n",0);
        g_iSendDataComplete = 1;  /* Set End of Transmission */
        if (0 != g_stUsbPipe.UsbSendData(NULL, 0))
        {
            g_iSendDataComplete = 1;
            PipeSendDataAfterRun();
        }
        return;
    }
    g_iSendDataComplete = 1;
    g_iSendDataFlag = 1;
    PipeSendDataAfterRun();
    printk("FATAL: HandleTxData(): Control Should not be here\n");
    return;
}


VOID PipeRxHandler (UCHAR *pucBuffer, UINT uiCount, INT iStatus)
{
    UINT uiLen = 0;
    UCHAR *pucRxData = NULL;
    INT iWriteIndex = 0;
    UINT uiBufIndex = 0;
    UINT uiRemain = 0;
    USHORT usPktId;
    USHORT usPktLen;
    PIPE_QUEUE_T *pstRecvQue = NULL;

    pucRxData  = pucBuffer;
    uiLen     = uiCount;

    if ((g_iPktHeadLen > uiLen))
    {
        return;
    }
    usPktId = pucBuffer[0] | (pucBuffer[1] << 8);
    usPktLen = pucBuffer[2] | (pucBuffer[3] << 8);
    pucRxData += g_iPktHeadLen;
    uiLen -= g_iPktHeadLen;
    pstRecvQue = Pipe_find(usPktId);
    if (NULL == pstRecvQue)
    {
        printk("WARNING: PipeRxhandler(). Not support for this pkt id %u.\n", usPktId);
        return;
    }

    /* Add to Queue if space available */
    spin_lock(&pstRecvQue->stLock);
    if (pstRecvQue->iCount == DATA_QUEUE_SIZE)
    {
        printk("WARNING: PipeRxhandler(). No Space to save data\n");
        spin_unlock(&pstRecvQue->stLock);
        return;
    }

    iWriteIndex = pstRecvQue->iWrIndex;

    if (pstRecvQue->bAbnormal)
    {
        /* 当前处于异常时，不处理数据 */
        printk("WARNING: PipeRxhandler(). in abnormal status\n");
        spin_unlock(&pstRecvQue->stLock);
        return;
    }
    else if (0 != iStatus)
    {
        pstRecvQue->bAbnormal = BOOL_TRUE;
        pstRecvQue->uiEntrySize[iWriteIndex] = MAX_PIPE_DATA_SIZE;    /* 大小为最大值时表示出错 */
        pstRecvQue->iCount++;
        wake_up_interruptible(&pstRecvQue->stWait);
        spin_unlock(&pstRecvQue->stLock);
        return;
    }

    /* Check if the Buffer has enough space */
    uiBufIndex = pstRecvQue->uiEntrySize[iWriteIndex];
    uiRemain = MAX_PIPE_DATA_SIZE - uiBufIndex;
    if (usPktLen > uiRemain)
    {
        printk("WARNING: PipeRxhandler(). More data than the buffer capacity. Data will be lost\n");
        usPktLen = uiRemain;
    }
    /* Write the data and length */
    pstRecvQue->uiEntrySize[iWriteIndex] += usPktLen;
    if (usPktLen != 0)
    {
        memcpy ((VOID*)&(pstRecvQue->aaucEntry[iWriteIndex][uiBufIndex]), pucRxData, usPktLen);
    }

    /* If end of transfer */
    if ((0 == usPktLen) || (usPktLen < MAX_ENDPOINT_SIZE - g_iPktHeadLen))
    {
        pstRecvQue->iWrIndex = (iWriteIndex + 1) % DATA_QUEUE_SIZE;
        pstRecvQue->iCount++;
        wake_up_interruptible(&pstRecvQue->stWait);
    }
    spin_unlock(&pstRecvQue->stLock);
    return;
}


STATIC INT RecvPipeDataFromSlave(PIPE_QUEUE_T *pstPipeQue, UCHAR *pucBuff, UINT uiTimeout)
{
    INT iLen = -1;
    LONG lTime = 0;
    INT iReadIndex = 0;

    if (!IsUsbDetected())
    {
        printk("ERROR: Calling RecvPipeData() without Pipe Initialized\n");
        return BINDER_PIPE_NOT_FOUND_USB_DEVICE;
    }

    lTime = msecs_to_jiffies(uiTimeout);
retryRead:
    spin_lock(&pstPipeQue->stLock);
    if (pstPipeQue->iCount)
    {
        iReadIndex = pstPipeQue->iRdIndex;
        iLen= pstPipeQue->uiEntrySize[iReadIndex];
        if ((iLen >0) && (MAX_PIPE_DATA_SIZE > iLen))
        {
            memcpy(pucBuff, (VOID*)pstPipeQue->aaucEntry[iReadIndex], iLen);
        }
        else
        {
           iLen = -1;
        }
        /* Reset Lenght which is used as Buffer Index in write */
        pstPipeQue->uiEntrySize[iReadIndex] = 0;
        pstPipeQue->iRdIndex = (iReadIndex + 1) % DATA_QUEUE_SIZE;
        pstPipeQue->iCount--;
        spin_unlock(&pstPipeQue->stLock);
    }
    else
    {
        spin_unlock(&pstPipeQue->stLock);
        if (0 == lTime)
        {
            if (wait_event_interruptible(pstPipeQue->stWait ,pstPipeQue->iCount))
            {
                printk("RecvPipeDataFromSlave is terminated by singal.\n");
                return BINDER_PIPE_WAIT_EVENT_TERMINATED;
            }
        }
        else
        {
            iLen = wait_event_interruptible_timeout(pstPipeQue->stWait ,pstPipeQue->iCount, lTime);
            if (iLen < 0)
            {
                printk("RecvPipeDataFromSlave is terminated by singal.\n");
                return BINDER_PIPE_WAIT_EVENT_TERMINATED;
            }
            if (0 == iLen)
            {
                printk("RecvPipeDataFromSlave timeout.\n");
                return BINDER_PIPE_WAIT_EVENT_TIMEOUT;
            }
        }
        goto retryRead;
    }
    if (iLen <= 0 )
    {
        goto retryRead;
    }
    DEBUG_PRINT("RecvPipeData(): Returning %d Bytes of Pipe data from host\n", iLen);
    return iLen;
}

STATIC INT SendPipeDataToSlave(UCHAR *pucBuff, UINT uiLen, UINT uiTimeout)
{
    INT   iRet = BINDER_SUCCESS;
    LONG lTime = 0;

    lTime = msecs_to_jiffies(uiTimeout);
    //DEBUG_PRINT("time 0x%x timeout 0x%x \n",time,timeout);
    if (!g_iSendDataFlag)
    {
        DEBUG_PRINT("INFO: SendPipeDataToSlave(): waiting Transmitting Data.\n");
        if (0 == lTime)
        {
            if(wait_event_interruptible(g_stPipeQueue.stWrWait ,g_iSendDataFlag))
            {
                printk("SendPipeDataToSlave is terminated by singal.\n");
                if (pucBuff)
                {
                    kfree(pucBuff);
                    pucBuff = NULL;
                }
                return BINDER_PIPE_WAIT_EVENT_TERMINATED;
            }
        }
        else
        {
            iRet = wait_event_interruptible_timeout(g_stPipeQueue.stWrWait ,g_iSendDataFlag, lTime);
            if (iRet < 0)
            {
                printk("SendPipeDataToSlave is terminated by singal.\n");
                if (pucBuff)
                {
                    kfree(pucBuff);
                    pucBuff = NULL;
                }
                return BINDER_PIPE_WAIT_EVENT_TERMINATED;
            }
            if (iRet == 0)
            {
                if (pucBuff)
                {
                    kfree(pucBuff);
                    pucBuff = NULL;
                }
                printk("SendPipeDataToSlave timeout.\n");
                return BINDER_PIPE_WAIT_EVENT_TIMEOUT;
            }
        }

        DEBUG_PRINT("INFO: SendPipeDataToSlave(): wake up Transmitting Data.\n");
    }

    g_pucTxDataBuf = pucBuff;
    g_uiTxDataLen = uiLen;
    g_uiTxDataPos = 0;

    g_iSendDataComplete = 0;
    g_iSendDataFlag = 0;
    PipeTxHandler();

    return iRet;
}

ssize_t PipeWriteByKernel(USHORT usId, const CHAR *pcBuf, size_t ulCount, UINT uiTimeout)
{
    CHAR *pcLocalBuffer = NULL;
    INT iBuflen = 0;

    /* 多申请一段内存, 用于存放包头信息和发送空数据包 */
    iBuflen = ((ulCount / MAX_ENDPOINT_SIZE) + 1) * MAX_ENDPOINT_SIZE;
    iBuflen += g_iPktHeadLen;
    pcLocalBuffer= (CHAR *)kmalloc(iBuflen, GFP_ATOMIC);
    if (NULL == pcLocalBuffer)
    {
        return -ENOMEM;
    }
    /* 自定义数据头结构:
     * -------------------------------------------------
     * |   BYTE0   |   BYTE1   |   BYTE2   |   BYTE3   |
     * -------------------------------------------------
     * |      session id       |       pkt len         |
     * -------------------------------------------------
     */
    /* 此处仅填充ID，在PipeTxHandler中填充实际pktlen*/
    pcLocalBuffer[0] = usId & 0xFF;
    pcLocalBuffer[1] = (usId >> 8) & 0xFF;

    memcpy(pcLocalBuffer + g_iPktHeadLen, pcBuf, ulCount);

    if (SendPipeDataToSlave(pcLocalBuffer, ulCount, uiTimeout) < 0)
    {
        printk("PipeWrite() failed,send pipe data failed.\n");
        return -EFAULT;
    }

    return ulCount;
}

ssize_t PipeRead(USHORT usId, CHAR *pcBuf, size_t ulCount, UINT uiTimeout)
{
    INT  iLen = 0;
    size_t ulRemain = ulCount;
    PIPE_QUEUE_T *pstPipeQue = NULL;
    CHAR *pcLocalBuffer = NULL;

    pstPipeQue = Pipe_find(usId);
    if (NULL == pstPipeQue)
    {
        return BINDER_IO_READ_ERROR;
    }

    pcLocalBuffer = (CHAR *)kmalloc(MAX_PIPE_DATA_SIZE, GFP_ATOMIC);
    if (NULL == pcLocalBuffer)
    {
        return -ENOMEM;
    }
    
    iLen = RecvPipeDataFromSlave(pstPipeQue, pcLocalBuffer, uiTimeout);
    if (iLen < 0)
    {
        msleep(300);  /* 等待本次数据发完，再清除管道数据，防止数据残留 */
        (VOID)ClearPipeDataQueue(pstPipeQue);
        kfree(pcLocalBuffer);
        return -EFAULT;
    }
    iLen = (iLen > ulRemain) ? ulRemain : iLen;
    if (copy_to_user(pcBuf, pcLocalBuffer, iLen))
    {
        kfree(pcLocalBuffer);
        return -EFAULT;
    }
    pcBuf += iLen;
    ulRemain -= iLen;
    kfree(pcLocalBuffer);
    
    return ulCount - ulRemain;
}

ssize_t PipeWrite(USHORT usId, const CHAR __user *pcBuf, size_t ulCount, UINT uiTimeout)
{
    CHAR *pcLocalBuffer = NULL;
    INT iBuflen = 0;

    /* 多申请一段内存, 用于存放包头信息和发送空数据包 */
    iBuflen = ((ulCount / MAX_ENDPOINT_SIZE) + 1) * MAX_ENDPOINT_SIZE;
    iBuflen += g_iPktHeadLen;
    pcLocalBuffer = (CHAR *)kmalloc(iBuflen, GFP_ATOMIC);
    if (NULL == pcLocalBuffer)
    {
        return -ENOMEM;
    }
    /* 自定义数据头结构:
     * -------------------------------------------------
     * |   BYTE0   |   BYTE1   |   BYTE2   |   BYTE3   |
     * -------------------------------------------------
     * |      session id       |       pkt len         |
     * -------------------------------------------------
     */
    /* 此处仅填充ID，在PipeTxHandler中填充实际pktlen*/
    pcLocalBuffer[0] = usId & 0xFF;
    pcLocalBuffer[1] = (usId >> 8) & 0xFF;

    if (copy_from_user(pcLocalBuffer + g_iPktHeadLen, pcBuf, ulCount))
    {
        kfree(pcLocalBuffer);
        printk("PipeWrite() failed,copy from user failed.\n");
        return -EFAULT;
    }

    if (SendPipeDataToSlave(pcLocalBuffer, ulCount, uiTimeout) < 0)
    {
        printk("PipeWrite() failed,send pipe data failed.\n");

        return -EFAULT;
    }

    return ulCount;
}

/* Note : Limit read and write sizes to max of 4K. Otherwise, host app and usb xmit timeout
	  happens. If bigger size needed, you need to tweak these sizes */

#ifdef __cplusplus
}
#endif /* __cplusplus */

