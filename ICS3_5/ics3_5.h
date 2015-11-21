/* ========================================
 *
 * Copyright YOUR COMPANY, THE YEAR
 * All Rights Reserved
 * UNPUBLISHED, LICENSED SOFTWARE.
 *
 * CONFIDENTIAL AND PROPRIETARY INFORMATION
 * WHICH IS THE PROPERTY OF your company.
 *
 * ========================================
*/

#ifndef ICS3_5_H
#define ICS3_5_H

#include <project.h>
    
/*
 * パラメータを書き込み、読み込みする際に引数に使用する
 * カッコ内は設定できる数値の範囲
 */
typedef enum {
    EEPROM,    // EEPROM 実装してないのでやめてね
    STRC,      // ストレッチ   (1~127)
    SPD,       // スピード    (1~127)
    CUR,       // 電流制限値 (1~63)
    TMP        // 温度上限   (1~127)
}Param;


/*
 * サーボの初期化関数
 * プログラム開始時に呼び出してね
 */
void Servo_Start();

/*
 * サーボのIDを設定する関数
 * 引数: int8 id    設定するID(0~31)
 * 返り値: int8    正常に動けば0, エラーなら-1
 */
int8 Servo_SetId(int8 id);

/*
 * サーボのIDを確認する関数
 * 返り値: int8    接続されているサーボのID, エラーなら-1
 */
int8 Servo_GetId();

/*
 * サーボの角度を制御する関数
 * 引数: int8 id    対象のサーボID値(0~31)
 *      uint16 value    設定する値
 * 返り値: int16    サーボから返ってきた角度値, エラーなら-1
 */
int16 Servo_SetPosition(int8 id, uint16 value);

/*
 * サーボの角度を度で設定する関数
 * 引数: int8 id    対象のサーボID値(0~31)
 *      int16 value    設定する度数(-135~+135)
 * 返り値: int16    サーボから返ってきた度数, エラーなら-1
 */
int16 Servo_SetAngle(int8 id,int16 angle);
/*
 * サーボの角度を度で確認する関数
 * 引数: int8 id    対象のサーボID値(0~31)
 * 返り値: int16    サーボから返ってきた度数, エラーなら-1
 */
int16 Servo_GetAngle(int8 id);

/*
 * サーボのパラメータを設定する関数
 * 引数: int8 id    対象のサーボID値(0~31)
 *      Param target    設定するパラメータを指定(Paramの定義を参照...)
 *      uint8 value    設定する値
 * 返り値: int8    設定したサーボから返ってきた値, エラーなら-1
 */
int8 Servo_SetParam(int8 id, Param target, uint8 value);

/*
 * サーボのパラメータを確認する関数
 * 引数: int8 id    対象のサーボID値(0~31)
 *      Param target    確認するパラメータを指定(Paramの定義を参照...)
 * 返り値: int8    サーボから返ってきた値, エラーなら-1
 */
int8 Servo_GetParam(int8 id, Param target);

#endif /* ICS3_5_H */

/* [] END OF FILE */
