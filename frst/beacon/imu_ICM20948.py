#!/usr/bin/python
# -*- coding:utf-8 -*-
import time
try:
  import smbus
except Exception:
  pass
import math


class ICM20948(object):
        def __init__(self,address=None):
                self.Gyro = [0, 0, 0]
                self.Accel = [0, 0, 0]
                self.Mag = [0, 0, 0]
                self.pitch = 0.0
                self.roll = 0.0
                self.yaw = 0.0
                self.pu8data = [0, 0, 0, 0, 0, 0, 0, 0]
                self.U8tempX = [0, 0, 0, 0, 0, 0, 0, 0, 0]
                self.U8tempY = [0, 0, 0, 0, 0, 0, 0, 0, 0]
                self.U8tempZ = [0, 0, 0, 0, 0, 0, 0, 0, 0]
                self.GyroOffset = [0, 0, 0]
                self.MotionVal = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

                self.Ki = 1.0
                self.Kp = 4.50
                self.q0 = 1.
                self.q1 = 0.
                self.q2 = 0.
                self.q3 = 0.

                self.angles = [0.0, 0.0, 0.0]
                self.true = 0x01
                self.false = 0x00
                # define ICM-20948 Device I2C address
                self.I2C_ADD_ICM20948 = 0x68
                self.I2C_ADD_ICM20948_AK09916 = 0x0C
                self.I2C_ADD_ICM20948_AK09916_READ = 0x80
                self.I2C_ADD_ICM20948_AK09916_WRITE = 0x00
                # define ICM-20948 Register
                # user bank 0 register
                self.REG_ADD_WIA = 0x00
                self.REG_VAL_WIA = 0xEA
                self.REG_ADD_USER_CTRL = 0x03
                self.REG_VAL_BIT_DMP_EN = 0x80
                self.REG_VAL_BIT_FIFO_EN = 0x40
                self.REG_VAL_BIT_I2C_MST_EN = 0x20
                self.REG_VAL_BIT_I2C_IF_DIS = 0x10
                self.REG_VAL_BIT_DMP_RST = 0x08
                self.REG_VAL_BIT_DIAMOND_DMP_RST = 0x04
                self.REG_ADD_PWR_MIGMT_1 = 0x06
                self.REG_VAL_ALL_RGE_RESET = 0x80
                self.REG_VAL_RUN_MODE = 0x01  # Non low-power mode
                self.REG_ADD_LP_CONFIG = 0x05
                self.REG_ADD_PWR_MGMT_1 = 0x06
                self.REG_ADD_PWR_MGMT_2 = 0x07
                self.REG_ADD_ACCEL_XOUT_H = 0x2D
                self.REG_ADD_ACCEL_XOUT_L = 0x2E
                self.REG_ADD_ACCEL_YOUT_H = 0x2F
                self.REG_ADD_ACCEL_YOUT_L = 0x30
                self.REG_ADD_ACCEL_ZOUT_H = 0x31
                self.REG_ADD_ACCEL_ZOUT_L = 0x32
                self.REG_ADD_GYRO_XOUT_H = 0x33
                self.REG_ADD_GYRO_XOUT_L = 0x34
                self.REG_ADD_GYRO_YOUT_H = 0x35
                self.REG_ADD_GYRO_YOUT_L = 0x36
                self.REG_ADD_GYRO_ZOUT_H = 0x37
                self.REG_ADD_GYRO_ZOUT_L = 0x38
                self.REG_ADD_EXT_SENS_DATA_00 = 0x3B
                self.REG_ADD_REG_BANK_SEL = 0x7F
                self.REG_VAL_REG_BANK_0 = 0x00
                self.REG_VAL_REG_BANK_1 = 0x10
                self.REG_VAL_REG_BANK_2 = 0x20
                self.REG_VAL_REG_BANK_3 = 0x30

                # user bank 1 register
                # user bank 2 register
                self.REG_ADD_GYRO_SMPLRT_DIV = 0x00
                self.REG_ADD_GYRO_CONFIG_1 = 0x01
                self.REG_VAL_BIT_GYRO_DLPCFG_2 = 0x10  # bit[5:3]
                self.REG_VAL_BIT_GYRO_DLPCFG_4 = 0x20  # bit[5:3]
                self.REG_VAL_BIT_GYRO_DLPCFG_6 = 0x30  # bit[5:3]
                self.REG_VAL_BIT_GYRO_FS_250DPS = 0x00  # bit[2:1]
                self.REG_VAL_BIT_GYRO_FS_500DPS = 0x02  # bit[2:1]
                self.REG_VAL_BIT_GYRO_FS_1000DPS = 0x04  # bit[2:1]
                self.REG_VAL_BIT_GYRO_FS_2000DPS = 0x06  # bit[2:1]
                self.REG_VAL_BIT_GYRO_DLPF = 0x01  # bit[0]
                self.REG_ADD_ACCEL_SMPLRT_DIV_2 = 0x11
                self.REG_ADD_ACCEL_CONFIG = 0x14
                self.REG_VAL_BIT_ACCEL_DLPCFG_2 = 0x10  # bit[5:3]
                self.REG_VAL_BIT_ACCEL_DLPCFG_4 = 0x20  # bit[5:3]
                self.REG_VAL_BIT_ACCEL_DLPCFG_6 = 0x30  # bit[5:3]
                self.REG_VAL_BIT_ACCEL_FS_2g = 0x00  # bit[2:1]
                self.REG_VAL_BIT_ACCEL_FS_4g = 0x02  # bit[2:1]
                self.REG_VAL_BIT_ACCEL_FS_8g = 0x04  # bit[2:1]
                self.REG_VAL_BIT_ACCEL_FS_16g = 0x06  # bit[2:1]
                self.REG_VAL_BIT_ACCEL_DLPF = 0x01  # bit[0]

                # user bank 3 register
                self.REG_ADD_I2C_SLV0_ADDR = 0x03
                self.REG_ADD_I2C_SLV0_REG = 0x04
                self.REG_ADD_I2C_SLV0_CTRL = 0x05
                self.REG_VAL_BIT_SLV0_EN = 0x80
                self.REG_VAL_BIT_MASK_LEN = 0x07
                self.REG_ADD_I2C_SLV0_DO = 0x06
                self.REG_ADD_I2C_SLV1_ADDR = 0x07
                self.REG_ADD_I2C_SLV1_REG = 0x08
                self.REG_ADD_I2C_SLV1_CTRL = 0x09
                self.REG_ADD_I2C_SLV1_DO = 0x0A

                # define ICM-20948 Register  end

                # define ICM-20948 MAG Register
                self.REG_ADD_MAG_WIA1 = 0x00
                self.REG_VAL_MAG_WIA1 = 0x48
                self.REG_ADD_MAG_WIA2 = 0x01
                self.REG_VAL_MAG_WIA2 = 0x09
                self.REG_ADD_MAG_ST2 = 0x10
                self.REG_ADD_MAG_DATA = 0x11
                self.REG_ADD_MAG_CNTL2 = 0x31
                self.REG_VAL_MAG_MODE_PD = 0x00
                self.REG_VAL_MAG_MODE_SM = 0x01
                self.REG_VAL_MAG_MODE_10HZ = 0x02
                self.REG_VAL_MAG_MODE_20HZ = 0x04
                self.REG_VAL_MAG_MODE_50HZ = 0x05
                self.REG_VAL_MAG_MODE_100HZ = 0x08
                self.REG_VAL_MAG_MODE_ST = 0x10
                # define ICM-20948 MAG Register  end

                self.MAG_DATA_LEN = 6

                self._address = address
                if self._address is None:
                  self._address = self.I2C_ADD_ICM20948

                self._bus = smbus.SMBus(1)
                bRet=self.icm20948Check()             #Initialization of the device multiple times after power on will result in a return error
                # while true != bRet:
                #   print("ICM-20948 Error\n" )
                #   time.sleep(0.5)
                # print("ICM-20948 OK\n" )
                time.sleep(0.5)                       #We can skip this detection by delaying it by 500 milliseconds
                # user bank 0 register
                self._write_byte( self.REG_ADD_REG_BANK_SEL , self.REG_VAL_REG_BANK_0)
                self._write_byte( self.REG_ADD_PWR_MIGMT_1 , self.REG_VAL_ALL_RGE_RESET)
                time.sleep(0.1)
                self._write_byte( self.REG_ADD_PWR_MIGMT_1 , self.REG_VAL_RUN_MODE)
                #user bank 2 register
                self._write_byte( self.REG_ADD_REG_BANK_SEL , self.REG_VAL_REG_BANK_2)
                self._write_byte( self.REG_ADD_GYRO_SMPLRT_DIV , 0x07)
                self._write_byte( self.REG_ADD_GYRO_CONFIG_1 , self.REG_VAL_BIT_GYRO_DLPCFG_6 | self.REG_VAL_BIT_GYRO_FS_1000DPS | self.REG_VAL_BIT_GYRO_DLPF)
                self._write_byte( self.REG_ADD_ACCEL_SMPLRT_DIV_2 ,  0x07)
                self._write_byte( self.REG_ADD_ACCEL_CONFIG , self.REG_VAL_BIT_ACCEL_DLPCFG_6 | self.REG_VAL_BIT_ACCEL_FS_2g | self.REG_VAL_BIT_ACCEL_DLPF)
                #user bank 0 register
                self._write_byte( self.REG_ADD_REG_BANK_SEL , self.REG_VAL_REG_BANK_0)
                time.sleep(0.1)
                self.icm20948GyroOffset()
                self.icm20948MagCheck()
                self.icm20948WriteSecondary( self.I2C_ADD_ICM20948_AK09916| self.I2C_ADD_ICM20948_AK09916_WRITE, self.REG_ADD_MAG_CNTL2, self.REG_VAL_MAG_MODE_20HZ)
                return

        def icm20948_Gyro_Accel_Read(self):
                self._write_byte( self.REG_ADD_REG_BANK_SEL , self.REG_VAL_REG_BANK_0)
                data =self._read_block(self.REG_ADD_ACCEL_XOUT_H, 12)
                self._write_byte( self.REG_ADD_REG_BANK_SEL , self.REG_VAL_REG_BANK_2)
                self.Accel[0] = (data[0]<<8)|data[1]
                self.Accel[1] = (data[2]<<8)|data[3]
                self.Accel[2] = (data[4]<<8)|data[5]
                self.Gyro[0]  = ((data[6]<<8)|data[7]) - self.GyroOffset[0]
                self.Gyro[1]  = ((data[8]<<8)|data[9]) - self.GyroOffset[1]
                self.Gyro[2]  = ((data[10]<<8)|data[11]) - self.GyroOffset[2]
                if self.Accel[0]>=32767:             #Solve the problem that Python shift will not overflow
                        self.Accel[0] = self.Accel[0]-65535
                elif self.Accel[0]<=-32767:
                        self.Accel[0] = self.Accel[0]+65535
                if self.Accel[1]>=32767:
                        self.Accel[1] = self.Accel[1]-65535
                elif self.Accel[1]<=-32767:
                        self.Accel[1] = self.Accel[1]+65535
                if self.Accel[2]>=32767:
                        self.Accel[2] = self.Accel[2]-65535
                elif self.Accel[2]<=-32767:
                        self.Accel[2] =self.Accel[2]+65535
                if self.Gyro[0]>=32767:
                        self.Gyro[0] = self.Gyro[0]-65535
                elif self.Gyro[0]<=-32767:
                        self.Gyro[0] = self.Gyro[0]+65535
                if self.Gyro[1]>=32767:
                        self.Gyro[1] = self.Gyro[1]-65535
                elif self.Gyro[1]<=-32767:
                        self.Gyro[1] = self.Gyro[1]+65535
                if self.Gyro[2]>=32767:
                        self.Gyro[2] = self.Gyro[2]-65535
                elif self.Gyro[2]<=-32767:
                        self.Gyro[2] = self.Gyro[2]+65535

                return

        def icm20948MagRead(self):
                counter=20
                while(counter>0):
                        time.sleep(0.01)
                        self.icm20948ReadSecondary( self.I2C_ADD_ICM20948_AK09916 | self.I2C_ADD_ICM20948_AK09916_READ , self.REG_ADD_MAG_ST2, 1)
                        if ((self.pu8data[0] & 0x01)!= 0):
                          break
                        counter-=1
                if counter!=0:
                        for i in range(0,8):
                                self.icm20948ReadSecondary( self.I2C_ADD_ICM20948_AK09916 | self.I2C_ADD_ICM20948_AK09916_READ, self.REG_ADD_MAG_DATA , self.MAG_DATA_LEN)
                                self.U8tempX[i] = (self.pu8data[1]<<8)|self.pu8data[0]
                                self.U8tempY[i] = (self.pu8data[3]<<8)|self.pu8data[2]
                                self.U8tempZ[i] = (self.pu8data[5]<<8)|self.pu8data[4]
                        self.Mag[0]=(self.U8tempX[0]+self.U8tempX[1]+self.U8tempX[2]+self.U8tempX[3]+self.U8tempX[4]+self.U8tempX[5]+self.U8tempX[6]+self.U8tempX[7])/8
                        self.Mag[1]=-(self.U8tempY[0]+self.U8tempY[1]+self.U8tempY[2]+self.U8tempY[3]+self.U8tempY[4]+self.U8tempY[5]+self.U8tempY[6]+self.U8tempY[7])/8
                        self.Mag[2]=-(self.U8tempZ[0]+self.U8tempZ[1]+self.U8tempZ[2]+self.U8tempZ[3]+self.U8tempZ[4]+self.U8tempZ[5]+self.U8tempZ[6]+self.U8tempZ[7])/8

                if self.Mag[0]>=32767:            #Solve the problem that Python shift will not overflow
                        self.Mag[0]=self.Mag[0]-65535
                elif self.Mag[0]<=-32767:
                        self.Mag[0]=self.Mag[0]+65535
                if self.Mag[1]>=32767:
                        self.Mag[1]=self.Mag[1]-65535
                elif self.Mag[1]<=-32767:
                        self.Mag[1]=self.Mag[1]+65535
                if self.Mag[2]>=32767:
                        self.Mag[2]=self.Mag[2]-65535
                elif self.Mag[2]<=-32767:
                        self.Mag[2]=self.Mag[2]+65535
                return

        def icm20948ReadSecondary(self, u8I2CAddr, u8RegAddr, u8Len):
                self._write_byte( self.REG_ADD_REG_BANK_SEL,  self.REG_VAL_REG_BANK_3) #swtich bank3
                self._write_byte( self.REG_ADD_I2C_SLV0_ADDR, u8I2CAddr)
                self._write_byte( self.REG_ADD_I2C_SLV0_REG,  u8RegAddr)
                self._write_byte( self.REG_ADD_I2C_SLV0_CTRL, self.REG_VAL_BIT_SLV0_EN | u8Len)

                self._write_byte( self.REG_ADD_REG_BANK_SEL, self.REG_VAL_REG_BANK_0) #swtich bank0

                u8Temp = self._read_byte(self.REG_ADD_USER_CTRL)
                u8Temp |= self.REG_VAL_BIT_I2C_MST_EN
                self._write_byte( self.REG_ADD_USER_CTRL, u8Temp)
                time.sleep(0.01)
                u8Temp &= ~self.REG_VAL_BIT_I2C_MST_EN
                self._write_byte( self.REG_ADD_USER_CTRL, u8Temp)

                for i in range(0, u8Len):
                        self.pu8data[i]= self._read_byte( self.REG_ADD_EXT_SENS_DATA_00+i)

                self._write_byte( self.REG_ADD_REG_BANK_SEL, self.REG_VAL_REG_BANK_3) #swtich bank3

                u8Temp = self._read_byte(self.REG_ADD_I2C_SLV0_CTRL)
                u8Temp &= ~((self.REG_VAL_BIT_I2C_MST_EN)&(self.REG_VAL_BIT_MASK_LEN))
                self._write_byte( self.REG_ADD_I2C_SLV0_CTRL,  u8Temp)

                self._write_byte( self.REG_ADD_REG_BANK_SEL, self.REG_VAL_REG_BANK_0) #swtich bank0
                return

        def icm20948WriteSecondary(self,u8I2CAddr,u8RegAddr,u8data):
                u8Temp=0
                self._write_byte( self.REG_ADD_REG_BANK_SEL,  self.REG_VAL_REG_BANK_3) #swtich bank3
                self._write_byte( self.REG_ADD_I2C_SLV1_ADDR, u8I2CAddr)
                self._write_byte( self.REG_ADD_I2C_SLV1_REG,  u8RegAddr)
                self._write_byte( self.REG_ADD_I2C_SLV1_DO,   u8data)
                self._write_byte( self.REG_ADD_I2C_SLV1_CTRL, self.REG_VAL_BIT_SLV0_EN|1)

                self._write_byte( self.REG_ADD_REG_BANK_SEL, self.REG_VAL_REG_BANK_0) #swtich bank0

                u8Temp = self._read_byte(self.REG_ADD_USER_CTRL)
                u8Temp |= self.REG_VAL_BIT_I2C_MST_EN
                self._write_byte( self.REG_ADD_USER_CTRL, u8Temp)
                time.sleep(0.01)
                u8Temp &= ~self.REG_VAL_BIT_I2C_MST_EN
                self._write_byte( self.REG_ADD_USER_CTRL, u8Temp)

                self._write_byte( self.REG_ADD_REG_BANK_SEL, self.REG_VAL_REG_BANK_3) #swtich bank3

                u8Temp = self._read_byte(self.REG_ADD_I2C_SLV0_CTRL)
                u8Temp &= ~((self.REG_VAL_BIT_I2C_MST_EN)&(self.REG_VAL_BIT_MASK_LEN))
                self._write_byte( self.REG_ADD_I2C_SLV0_CTRL,  u8Temp)

                self._write_byte( self.REG_ADD_REG_BANK_SEL, self.REG_VAL_REG_BANK_0) #swtich bank0

                return

        def icm20948GyroOffset(self):
                s32TempGx = 0
                s32TempGy = 0
                s32TempGz = 0
                for i in range(0,32):
                        self.icm20948_Gyro_Accel_Read()
                        s32TempGx += self.Gyro[0]
                        s32TempGy += self.Gyro[1]
                        s32TempGz += self.Gyro[2]
                        time.sleep(0.01)

                self.GyroOffset[0] = s32TempGx >> 5
                self.GyroOffset[1] = s32TempGy >> 5
                self.GyroOffset[2] = s32TempGz >> 5
                return

        def _read_byte(self,cmd):
                return self._bus.read_byte_data(self._address,cmd)

        def _read_block(self, reg, length=1):
                return self._bus.read_i2c_block_data(self._address, reg, length)

        def _read_u16(self,cmd):
                LSB = self._bus.read_byte_data(self._address,cmd)
                MSB = self._bus.read_byte_data(self._address,cmd+1)
                return (MSB	<< 8) + LSB

        def _write_byte(self,cmd,val):
                self._bus.write_byte_data(self._address,cmd,val)
                time.sleep(0.0001)
                return

        def imuAHRSupdate(self, gx, gy, gz, ax, ay, az, mx, my, mz):
                norm=0.0
                hx = hy = hz = bx = bz = 0.0
                vx = vy = vz = wx = wy = wz = 0.0
                exInt = eyInt = ezInt = 0.0
                ex=ey=ez=0.0
                halfT = 0.024

                q0q0 = self.q0 * self.q0
                q0q1 = self.q0 * self.q1
                q0q2 = self.q0 * self.q2
                q0q3 = self.q0 * self.q3
                q1q1 = self.q1 * self.q1
                q1q2 = self.q1 * self.q2
                q1q3 = self.q1 * self.q3
                q2q2 = self.q2 * self.q2
                q2q3 = self.q2 * self.q3
                q3q3 = self.q3 * self.q3

                norm = float(1/math.sqrt(ax * ax + ay * ay + az * az))
                ax = ax * norm
                ay = ay * norm
                az = az * norm

                norm = float(1/math.sqrt(mx * mx + my * my + mz * mz))
                mx = mx * norm
                my = my * norm
                mz = mz * norm

                # compute reference direction of flux
                hx = 2 * mx * (0.5 - q2q2 - q3q3) + 2 * my * (q1q2 - q0q3) + 2 * mz * (q1q3 + q0q2)
                hy = 2 * mx * (q1q2 + q0q3) + 2 * my * (0.5 - q1q1 - q3q3) + 2 * mz * (q2q3 - q0q1)
                hz = 2 * mx * (q1q3 - q0q2) + 2 * my * (q2q3 + q0q1) + 2 * mz * (0.5 - q1q1 - q2q2)
                bx = math.sqrt((hx * hx) + (hy * hy))
                bz = hz

                # estimated direction of gravity and flux (v and w)
                vx = 2 * (q1q3 - q0q2)
                vy = 2 * (q0q1 + q2q3)
                vz = q0q0 - q1q1 - q2q2 + q3q3
                wx = 2 * bx * (0.5 - q2q2 - q3q3) + 2 * bz * (q1q3 - q0q2)
                wy = 2 * bx * (q1q2 - q0q3) + 2 * bz * (q0q1 + q2q3)
                wz = 2 * bx * (q0q2 + q1q3) + 2 * bz * (0.5 - q1q1 - q2q2)

                # error is sum of cross product between reference direction of fields and direction measured by sensors
                ex = (ay * vz - az * vy) + (my * wz - mz * wy)
                ey = (az * vx - ax * vz) + (mz * wx - mx * wz)
                ez = (ax * vy - ay * vx) + (mx * wy - my * wx)

                if (ex != 0.0 and ey != 0.0 and ez != 0.0):
                        exInt = exInt + ex * self.Ki * halfT
                        eyInt = eyInt + ey * self.Ki * halfT
                        ezInt = ezInt + ez * self.Ki * halfT

                gx = gx + self.Kp * ex + exInt
                gy = gy + self.Kp * ey + eyInt
                gz = gz + self.Kp * ez + ezInt

                self.q0 = self.q0 + (-self.q1 * gx - self.q2 * gy - self.q3 * gz) * halfT
                self.q1 = self.q1 + (self.q0 * gx + self.q2 * gz - self.q3 * gy) * halfT
                self.q2 = self.q2 + (self.q0 * gy - self.q1 * gz + self.q3 * gx) * halfT
                self.q3 = self.q3 + (self.q0 * gz + self.q1 * gy - self.q2 * gx) * halfT

                norm = float(1/math.sqrt(self.q0 * self.q0 + self.q1 * self.q1 + self.q2 * self.q2 + self.q3 * self.q3))
                self.q0 = self.q0 * norm
                self.q1 = self.q1 * norm
                self.q2 = self.q2 * norm
                self.q3 = self.q3 * norm
                return

        def icm20948Check(self):
                bRet=self.false
                if self.REG_VAL_WIA == self._read_byte(self.REG_ADD_WIA):
                  bRet = self.true
                return bRet

        def icm20948MagCheck(self):
                self.icm20948ReadSecondary( self.I2C_ADD_ICM20948_AK09916 | self.I2C_ADD_ICM20948_AK09916_READ, self.REG_ADD_MAG_WIA1, 2)
                if (self.pu8data[0] == self.REG_VAL_MAG_WIA1) and ( self.pu8data[1] == self.REG_VAL_MAG_WIA2) :
                    bRet = self.true
                    return bRet

        def icm20948CalAvgValue(self):
                self.MotionVal[0] = self.Gyro[0]/32.8
                self.MotionVal[1] = self.Gyro[1]/32.8
                self.MotionVal[2] = self.Gyro[2]/32.8
                self.MotionVal[3] = self.Accel[0]
                self.MotionVal[4] = self.Accel[1]
                self.MotionVal[5] = self.Accel[2]
                self.MotionVal[6] = self.Mag[0]
                self.MotionVal[7] = self.Mag[1]
                self.MotionVal[8] = self.Mag[2]
    
if __name__ == '__main__':
        print("\nSense HAT Test Program ...\n")
        icm20948=ICM20948()

        with open('./magneto_test_data.txt', 'w') as f:
                init_ts = time.time()
                while True:
                  icm20948.icm20948_Gyro_Accel_Read()
                  icm20948.icm20948MagRead()
                  icm20948.icm20948CalAvgValue()
                  time.sleep(0.1)
                  icm20948.imuAHRSupdate(icm20948.MotionVal[0] * 0.0175, icm20948.MotionVal[1] * 0.0175,icm20948.MotionVal[2] * 0.0175,
                              icm20948.MotionVal[3],icm20948.MotionVal[4],icm20948.MotionVal[5],
                              icm20948.MotionVal[6], icm20948.MotionVal[7], icm20948.MotionVal[8])

                  pitch = math.asin(-2 * icm20948.q1 * icm20948.q3 + 2 * icm20948.q0 * icm20948.q2) * 57.3
                  roll = math.atan2(2 * icm20948.q2 * icm20948.q3 + 2 * icm20948.q0 * icm20948.q1, -2 * icm20948.q1 * icm20948.q1 - 2 * icm20948.q2 * icm20948.q2 + 1) * 57.3
                  yaw = math.atan2(-2 * icm20948.q1 * icm20948.q2 - 2 * icm20948.q0 * icm20948.q3, 2 * icm20948.q2 * icm20948.q2 + 2 * icm20948.q3 * icm20948.q3 - 1) * 57.3

                  print("\r\n /-------------------------------------------------------------/ \r\n")
                  # print('\r\n Roll = %.2f , Pitch = %.2f , Yaw = %.2f\r\n'%(icm20948.roll, icm20948.pitch, icm20948.yaw))
                  # print('\r\nAcceleration:  X = %d , Y = %d , Z = %d\r\n'%(icm20948.Accel[0], icm20948.Accel[1], icm20948.Accel[2]))
                  # print('\r\nGyroscope:     X = %d , Y = %d , Z = %d\r\n'%(icm20948.Gyro[0], icm20948.Gyro[1], icm20948.Gyro[2]))
                  etime = round(time.time() - init_ts, 3)
                  ftext = str(etime)+", "+str((icm20948.Mag[0]))+", "+str((icm20948.Mag[1]))+", "+str((icm20948.Mag[2]))
                  f.write(ftext+'\n')
                  print(ftext)




       
      
         
      





      
