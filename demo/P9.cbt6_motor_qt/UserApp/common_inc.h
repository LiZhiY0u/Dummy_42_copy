#ifndef __LOOP_H
#define __LOOP_H

#ifdef __cplusplus
extern "C"
{
#endif
/*------------- C Scope -------------*/
#include "stdint.h"
#include "main.h"
#include "tim.h"
#include "usart.h"
#include "can.h"
#include "gpio.h"

    void Main(void);

    void OnCanCmd(uint8_t _cmd, uint8_t *_data, uint32_t _len);

    void tim4callback(void);

    void test2();

    void test1();

#ifdef __cplusplus
}
/*-------------- C++ Scope --------*/
/*cpp用的和带<cstdint>的放下面*/
#include <cstdio>
#include "motor.h"
#include "mt6816_base.h"
#include "encoder_base.h"
#include "tb67h450_base.h"
#include "encoder_calibrator_base.h"
#include "button_base.h"
#include "uart_handle.h"
#include "uart_protocol.h"

// C++测试
class Test
{
public:
    Test()
    {
    }
    explicit Test(uint8_t _id) : id(_id)
    {
    }
    enum Event
    {
    };
    void tick(void);
    uint16_t num;

protected:
    uint8_t id;

private:
    bool flag;
};
#endif
#endif

/**
 *                             _ooOoo_
 *                            o8888888o
 *                            88" . "88
 *                            (| -_- |)
 *                            O\  =  /O
 *                         ____/`---'\____
 *                       .'  \\|     |//  `.
 *                      /  \\|||  :  |||//  \
 *                     /  _||||| -:- |||||-  \
 *                     |   | \\\  -  /// |   |
 *                     | \_|  ''\---/''  |   |
 *                     \  .-\__  `-`  ___/-. /
 *                   ___`. .'  /--.--\  `. . __
 *                ."" '<  `.___\_<|>_/___.'  >'"".
 *               | | :  `- \`.;`\ _ /`;.`/ - ` : | |
 *               \  \ `-.   \_ __\ /__ _/   .-` /  /
 *          ======`-.____`-.___\_____/___.-`____.-'======
 *                             `=---='
 *          ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
 *                     佛祖保佑        永无BUG
 *            佛曰:
 *                   写字楼里写字间，写字间里程序员；
 *                   程序人员写程序，又拿程序换酒钱。
 *                   酒醒只在网上坐，酒醉还来网下眠；
 *                   酒醉酒醒日复日，网上网下年复年。
 *                   但愿老死电脑间，不愿鞠躬老板前；
 *                   奔驰宝马贵者趣，公交自行程序员。
 *                   别人笑我忒疯癫，我笑自己命太贱；
 *                   不见满街漂亮妹，哪个归得程序员？
 */
