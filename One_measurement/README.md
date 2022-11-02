# 一路ADC的测量
## 思路

使用STM32的DAC输出一个常量，并添加随机噪声作为干扰量，单片机通过串口接收这个量并实时显示波形图。

使用ADC测量该变量，使用卡尔曼滤波进行估计，逐次迭代消除干扰量，趋近于稳定值。

## 具体步骤
先创建工程并移植DAC驱动，然后完成ADC和DAC设备的初始化和注册。控制DAC1，并在其寄存器中写3000，用万用表测得PA4的输出电平为2.42V。使用ADC1（PC3）测DAC1的电平，每次测量都会有一定程度的误差，一般我们会使用一个低通滤波器（均值）进行滤波，得到更准确的数值，但其实我们可以使用卡尔曼滤波来不断预测更新，得到那个更接近真实值的电平。

### 测试DAC和ADC

先测一下ADC和DAC的功能是否正常，把PC3和PA4连接到一起，DAC写入的值是3000，PA4输出用万用表测试是2.42V。

```c
#include <rtthread.h>
#include <rtdevice.h>
#include <stdlib.h>
#define DAC_DEV_NAME        "dac1"  /* DAC 设备名称 */
#define DAC_DEV_CHANNEL     1       /* DAC 通道 */
#define REFER_VOLTAGE       330         /* 参考电压 3.3V,数据精度乘以100保留2位小数*/
#define CONVERT_BITS        (1 << 12)   /* 转换位数为12位 */

#define ADC_DEV_NAME        "adc1"      /* ADC 设备名称 */
#define ADC_DEV_CHANNEL     13           /* ADC 通道 */

static int vol_read_sample(int argc, char *argv[])
{
    rt_dac_device_t dac_dev;
    rt_adc_device_t adc_dev;
    rt_uint32_t value, vol;
    rt_err_t ret = RT_EOK;
    rt_uint8_t cnt = 0;

    /* ----------------- DAC 设备初始化 ----------------- */
    /* 查找设备 */
    dac_dev = (rt_dac_device_t)rt_device_find(DAC_DEV_NAME);
    if (dac_dev == RT_NULL)
    {
        rt_kprintf("dac sample run failed! can't find %s device!\n", DAC_DEV_NAME);
        return RT_ERROR;
    }

    /* 打开通道 */
    ret = rt_dac_enable(dac_dev, DAC_DEV_CHANNEL);

    /* 设置输出值 */
    value = 3000;
    rt_dac_write(dac_dev, DAC_DEV_CHANNEL, value);
    rt_kprintf("the value is :%d \n", value);

    /* 转换为对应电压值 */
    vol = value * REFER_VOLTAGE / CONVERT_BITS;
    rt_kprintf("the voltage is :%d.%02d \n", vol / 100, vol % 100);

    /* ----------------- ADC 设备初始化 ----------------- */

    /* 查找设备 */
    adc_dev = (rt_adc_device_t)rt_device_find(ADC_DEV_NAME);
    if (adc_dev == RT_NULL)
    {
        rt_kprintf("adc sample run failed! can't find %s device!\n", ADC_DEV_NAME);
        return RT_ERROR;
    }

    /* 使能设备 */
    ret = rt_adc_enable(adc_dev, ADC_DEV_CHANNEL);

    for(cnt = 0; cnt < 10; cnt++)
    {
        /* 读取ADC值 */
        value = rt_adc_read(adc_dev, ADC_DEV_CHANNEL);
        rt_kprintf("the adc value is :%d \n", value);

        /* 转换为对应电压值 */
        vol = value * REFER_VOLTAGE / CONVERT_BITS;
        rt_kprintf("the adc voltage is :%d.%02d \n", vol / 100, vol % 100);
        rt_thread_mdelay(1000);
    }

    return ret;
}
/* 导出到 msh 命令列表中 */
MSH_CMD_EXPORT(vol_read_sample, voltage convert and read sample);
```

使用ADC读取引脚电平，得到的数字量始终存在一定误差，可以使用卡尔曼滤波进行消除。

### 实现卡尔曼滤波

[STM32应用(六)一阶卡尔曼滤波代码和简单应用_三木今天学习了嘛的博客-CSDN博客_一阶卡尔曼滤波](https://blog.csdn.net/weixin_45751396/article/details/119595886)

教程中实现了简单的一维卡尔曼滤波，主要是设置先验估计协方差P和测量协方差Q的初始值，预测协方差P为1，说明刚开始的时候不相信预测值，因为都还没有测几个数据，肯定是更相信测的值，随着测的数越来越多，P也不断迭代，会逐步升高并稳定到一个值。

然后将上一次的P和固定的测量协方差Q相加，得到当前的P。

卡尔曼增益方程：卡尔曼增益 = k时刻系统估算协方差 / （k时刻系统估算协方差 + 观测噪声协方差）

之后就是利用已知的卡尔曼增益求出预测值和测量值的融合结果，作为当前的最佳估计值。

最后再使用公式更新协方差误差，这样就让卡尔曼增益和协方差误差不断更新，得到最佳的融合值。

```c
#include "Kalman.h"

void Kalman_Init()
{
	kfp.Last_P = 1;
	kfp.Now_P = 0;
	kfp.out = 0;
	kfp.Kg = 0;
	kfp.Q = 0;
	kfp.R = 0.01;
}

/**
 *卡尔曼滤波器
 *@param 	Kalman *kfp 卡尔曼结构体参数
 *   			float input 需要滤波的参数的测量值（即传感器的采集值）
 *@return 滤波后的参数（最优值）
 */
float KalmanFilter(Kalman *kfp,float input)
{
   //预测协方差方程：k时刻系统估算协方差 = k-1时刻的系统协方差 + 过程噪声协方差
   kfp->Now_P = kfp->Last_P + kfp->Q;
   //卡尔曼增益方程：卡尔曼增益 = k时刻系统估算协方差 / （k时刻系统估算协方差 + 观测噪声协方差）
   kfp->Kg = kfp->Now_P / (kfp->Now_P + kfp->R);
   //更新最优值方程：k时刻状态变量的最优值 = 状态变量的预测值 + 卡尔曼增益 * （测量值 - 状态变量的预测值）
   kfp->out = kfp->out + kfp->Kg * (input -kfp->out);//因为这一次的预测值就是上一次的输出值
   //更新协方差方程: 本次的系统协方差付给 kfp->LastP 威下一次运算准备。
   kfp->Last_P = (1-kfp->Kg) * kfp->Now_P;
   return kfp->out;
}
```

使用vofa串口示波器，显示波形，观察可以发现，直接采用ADC读取的数值，存在较大的误差，但是使用卡尔曼滤波器，通过融合预测值和采样值，逐渐得到最佳的估计值，最终卡尔曼滤波器的值稳定在2.41V，与万用表测得的2.42V基本一致。