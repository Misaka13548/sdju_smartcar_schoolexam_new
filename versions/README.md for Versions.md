## README.md for Versions

### main.c (大可能完赛版)

该版本基于 gjh 直接传给我的版本 实车跑过的大概率完赛版本 有问题直接上这个

### main.c (大可能完赛版 可视化修复版)

该版本基于 gjh 直接传给我的版本 做了代码可读性修复

### main.c (大可能完赛版 可视化修复版 逻辑修复版)

该版本基于 gjh 直接传给我做了代码可读性修复的版本 进一步修复了出左环岛失效的问题

以下是修改内容

#### 1.修改宏定义变量

先前的宏定义

```
#define change_steer3       (3)
#define change_steer4       (4)
```

现在的宏定义

```
#define change_steer4       (4)
#define change_steer5       (5)
```

修改了变量名 和 宏对应的参数

<br>

#### 2.修改宏定义参与的逻辑

先前的逻辑

```
case xh_SIGNAL:
    change_steer1 = 3;

    /* 出环岛判断 */
    if (

            ( (int32)inductor_now[1] > (int32)rLOOP_exit_ADC )
            && 
            ( (int32)inductor_now[0] + (int32)inductor_now[3] > (int32)LOOP_IN_ADC )

        )
    {
        state = change_steer3;
    }
    else if (

            ( (int32)inductor_now[2] > (int32)lLOOP_exit_ADC )
            && 
            ( (int32)inductor_now[0] + (int32)inductor_now[3] > (int32)LOOP_IN_ADC) 

        )
    {
        state = change_steer4;
    }
    
     break;
```

```
case change_steer3:
    change_steer1 = 3;
    state = cool_state;     // 进入状态机冷却状态，锁定状态机

    break;

case change_steer4:
    change_steer1 = 4;
    state = cool_state;     // 进入状态机冷却状态，锁定状态机

    break;
```

现在的逻辑

```
case xh_SIGNAL:
    change_steer1 = 3;

    /* 出环岛判断 */
    if (

            ( (int32)inductor_now[1] > (int32)rLOOP_exit_ADC )
            && 
            ( (int32)inductor_now[0] + (int32)inductor_now[3] > (int32)LOOP_IN_ADC )

        )
    {
        state = change_steer4;
    }
    else if (

            ( (int32)inductor_now[2] > (int32)lLOOP_exit_ADC )
            && 
            ( (int32)inductor_now[0] + (int32)inductor_now[3] > (int32)LOOP_IN_ADC) 

        )
    {
        state = change_steer5;
    }

    break;
```

```
 case change_steer4:
     change_steer1 = 4;
     state = cool_state;     // 进入状态机冷却状态，锁定状态机

     break;

 case change_steer5:
     change_steer1 = 5;
     state = cool_state;     // 进入状态机冷却状态，锁定状态机

     break;
```

<br>

修改了 变量名

#### 3.删除了死宏

```
#define EXIT_LOOP_SIGNAL    (5)
#define COOLDOWN_SIGNAL     (6)
```





~~求求你了做做版本管理吧，我真的很头晕喵，求求你了喵，我很难受喵~~

~~你要是不想用git你手动复制也可以的喵 T_T~~

~~我已经完全无从下手了，这里只能用文件夹分version了~~
