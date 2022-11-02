# 两路ADC读取
cubemx中初始化两路adc，生成初始化的代码。
添加驱动文件drv，修改配置文件kconfig和rt_config.h，注册adc1和ac2，让rtt支持adc。
添加adc的测试代码，测试adc的读取。
## kalman
此处可以使用二阶的卡尔曼滤波器，以两路adc的读取值为输入，输出为滤波后的值。程序实现可以调用两个kalman滤波的函数，求得滤波后的两路adc的平均值。