＃张

#用于使用两个步进电机的2自由度云台的程序
#采用结构体存储步进电机参数
#在定时器中断中完成脉冲的输出，通过存储在电机结构体数组里的当前位置（NowPostion或NowTangle）和目标位置（Goalpostion和Goaltangle）进行对比，并输出对应的定时器模拟PWM波。
#本例中采用了4个普通电机（无角度反馈的电机）+4个角度电机（有角度反馈的电机），两种电机的结构不同，转动时边界判定的原理不同
