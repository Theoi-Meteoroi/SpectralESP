""" MicroPython driver for the AS7265x 18-channel spectral sensor
    Edit by : Phumiphat Charoentananuwat"""

from micropython import const
import time
import struct
from Device import Device

AS7265X_ADDR = 0x49 #7-bit unshifted default I2C Address

AS7265X_SLAVE_STATUS_REG = 0x00
AS7265X_SLAVE_WRITE_REG  = 0X01
AS7265X_SLAVE_READ_REG   = 0x02

AS7265X_SLAVE_TX_VALID   = 0x02
AS7265X_SLAVE_RX_VALID   = 0x01

#Register addresses
AS7265X_HW_VERSION_HIGH  = 0x00
AS7265X_HW_VERSION_LOW   = 0x01

AS7265X_FW_VERSION_HIGH = 0x02
AS7265X_FW_VERSION_LOW  = 0x03

AS7265X_CONFIG            = 0x04
AS7265X_INTERGRATION_TIME = 0x05
AS7265X_DEVICE_TEMP       = 0x06
AS7265X_LED_CONFIG        = 0x07

#Raw channel registers
AS7265X_R_G_A    = 0x08
AS7265X_S_H_B    = 0x0A
AS7265X_T_I_C    = 0x0C
AS7265X_U_J_D    = 0x0E
AS7265X_V_K_E    = 0x10
AS7265X_W_L_F    = 0x12

#Calibrated channel registers
AS7265X_R_G_A_CAL =  0x14
AS7265X_S_H_B_CAL =  0x18
AS7265X_T_I_C_CAL =  0x1C
AS7265X_U_J_D_CAL =  0x20
AS7265X_V_K_E_CAL =  0x24
AS7265X_W_L_F_CAL =  0x28

AS7265X_DEV_SELECT_CONTROL = 0x4F

AS7265X_COEF_DATA_0      = 0x50
AS7265X_COEF_DATA_1      = 0x51
AS7265X_COEF_DATA_2      = 0x52
AS7265X_COEF_DATA_3      = 0x53
AS7265X_COEF_DATA_READ   = 0x54
AS7265X_COEF_DATA_WRITE  = 0x55

#Settings 

AS7265X_POLLING_DELAY = 5 #Amount of ms to wait between checking for virtual register changes

AS72651_NIR      =  0x00
AS72652_VISIBLE  =  0x01
AS72653_UV       =  0x02

AS7265x_LED_WHITE =	0x00 #White LED is connected to x51
AS7265x_LED_IR =	0x01 #IR LED is connected to x52
AS7265x_LED_UV =	0x02 #UV LED is connected to x53

AS7265X_LED_CURRENT_LIMIT_12_5MA  = 0b00
AS7265X_LED_CURRENT_LIMIT_25MA    = 0b01
AS7265X_LED_CURRENT_LIMIT_50MA    = 0b10
AS7265X_LED_CURRENT_LIMIT_100MA   = 0b11

AS7265X_INDICATOR_CURRENT_LIMIT_1MA   = 0b00
AS7265X_INDICATOR_CURRENT_LIMIT_2MA   = 0b01
AS7265X_INDICATOR_CURRENT_LIMIT_4MA   = 0b10
AS7265X_INDICATOR_CURRENT_LIMIT_8MA   = 0b11

AS7265X_GAIN_1X    = 0b00
AS7265X_GAIN_37X   = 0b01
AS7265X_GAIN_16X   = 0b10
AS7265X_GAIN_64X   = 0b11

AS7265X_MEASUREMENT_MODE_4CHAN             = 0b00
AS7265X_MEASUREMENT_MODE_4CHAN_2           = 0b01
AS7265X_MEASUREMENT_MODE_6CHAN_CONTINUOUS  = 0b10
AS7265X_MEASUREMENT_MODE_6CHAN_ONE_SHOT    = 0b11

class AS7265X:
    def __init__(self,i2c,**kwargs):
        self._AS7265X_ADDR = AS7265X_ADDR

        self._mode = AS7265X_MEASUREMENT_MODE_6CHAN_ONE_SHOT #Mode 4
        # TODO: Sanitize gain and integration time values
        self._gain = AS7265X_GAIN_64X # 64x
        self._integration_time = AS7265X_POLLING_DELAY
        self._sensor_version = 0
        #Create I2C device
        if i2c is None:
            raise ValueError('An I2C object is required.')
        self._device = Device(AS7265X_ADDR, i2c)
        self._i2c = i2c
        #Check and initialize device        
        #self.init_device()


    #Read a virtual register from the AS7265x
    def virtual_read_register(self,virtual_address):
        # Do a prelim check of the read register
        status = self._device.readU8(AS7265X_SLAVE_STATUS_REG);
        if((status & AS7265X_SLAVE_RX_VALID) != 0): # There is data to be read
            _ = self._device.readU8(AS7265X_SLAVE_READ_REG) # Read the byte but do nothing with it
        
        # Wait for WRITE register to be empty
        while True:
            status = self._device.readU8(AS7265X_SLAVE_STATUS_REG)
            if (status & AS7265X_SLAVE_TX_VALID) == 0:
                break # No inbound TX pending at slave. Okay to write now.
            time.sleep_ms(AS7265X_POLLING_DELAY)
        
        # Send the virtual register address (bit 7 should be 0 to indicate we are reading a register)
        self._device.write8(AS7265X_SLAVE_WRITE_REG , virtual_address)
        
        # Wait for READ flag to be set
        while True:
            status = self._device.readU8(AS7265X_SLAVE_STATUS_REG)
            if((status & AS7265X_SLAVE_RX_VALID) != 0): # Data is ready
                break # No inbound TX pending at slave. Okay to write now.
            time.sleep_ms(AS7265X_POLLING_DELAY)
        
        result = self._device.readU8(AS7265X_SLAVE_READ_REG)
        return result

    
    #Write to a virtual register in the AS7265x
    def virtual_write_register(self,virtual_address,value):
        #Wait for WRITE register to be empty
        while True:
            status = self._device.readU8(AS7265X_SLAVE_STATUS_REG)
            if((status & AS7265X_SLAVE_TX_VALID) == 0):
                break #No inbound TX pending at slave. Okay to write now.
            time.sleep_ms(AS7265X_POLLING_DELAY)
        
        #Send the virtual register address (setting bit 7 to indicate we are writing to a register).
        self._device.write8(AS7265X_SLAVE_WRITE_REG , (virtual_address | 0x80))

        # Wait for Write register to be empty
        while True:
            status = self._device.readU8(AS7265X_SLAVE_STATUS_REG)
            if((status & AS7265X_SLAVE_TX_VALID) == 0):
                break # No inbound TX pending at slave. Okay to write now.
            time.sleep_ms(AS7265X_POLLING_DELAY)
        
        # Sendthe data to complete the operation.
        self._device.write8(AS7265X_SLAVE_WRITE_REG , value)

    def get_devicetype(self):
        return self.virtual_read_register(AS7265X_HW_VERSION_HIGH)
    
    def get_hardware_version(self):
        return self.virtual_read_register(AS7265X_HW_VERSION_LOW)
    
    def get_major_firmware_version(self):
        self.virtual_write_register(AS7265X_FW_VERSION_HIGH , 0x01) #Set to 0x01 for Major
        self.virtual_write_register(AS7265X_FW_VERSION_LOW  , 0x01) #Set to 0x01 for Major
        return self.virtual_read_register(AS7265X_FW_VERSION_LOW)
    
    def get_patch_firmware_version(self):
        self.virtual_write_register(AS7265X_FW_VERSION_HIGH , 0x02) #Set to 0x02 for Patch
        self.virtual_write_register(AS7265X_FW_VERSION_LOW  , 0x02) #Set to 0x02 for Patch
        return self.virtual_read_register(AS7265X_FW_VERSION_LOW)
    
    def get_build_firmware_version(self):
        self.virtual_write_register(AS7265X_FW_VERSION_HIGH , 0x03) #Set to 0x03 for Build
        self.virtual_write_register(AS7265X_FW_VERSION_LOW  , 0x03) #Set to 0x03 for Build
        return self.virtual_read_register(AS7265X_FW_VERSION_LOW)
    
    #Tell IC to take all channel measurements and polls for data ready flag
    def take_measurements(self):
        #Clear DATA_RDY flag when using mode 3
        self.clear_data_available()
        self.set_measurement_mode(AS7265X_MEASUREMENT_MODE_6CHAN_ONE_SHOT) #Set mode to all 6-channels , one shot
        #Wait for data to be ready
        while self.data_available() == False:
            time.sleep_ms(AS7265X_POLLING_DELAY)
        #Readings can now be accessed via get_calibrate_A() , get_J() , etc
    
    #Turns on all bulbs, takes measurements of all channels, turns off all bulbs
    def take_measurements_with_bulb(self):
        #Turns on bulb
        self.enable_bulb(AS7265x_LED_WHITE)
        self.enable_bulb(AS7265x_LED_IR)
        self.enable_bulb(AS7265x_LED_UV)
        #take measure
        self.take_measurements()
        #Turns off bulb
        self.disable_bulb(AS7265x_LED_WHITE)
        self.disable_bulb(AS7265x_LED_IR)
        self.disable_bulb(AS7265x_LED_UV)
    
    #Get the various  color readings
    def get_G(self):
        return self.get_channel(AS7265X_R_G_A,AS72652_VISIBLE)
    def get_H(self):
        return self.get_channel(AS7265X_S_H_B,AS72652_VISIBLE)
    def get_I(self):
        return self.get_channel(AS7265X_T_I_C,AS72652_VISIBLE)
    def get_J(self):
        return self.get_channel(AS7265X_U_J_D,AS72652_VISIBLE)
    def get_K(self):
        return self.get_channel(AS7265X_V_K_E,AS72652_VISIBLE)
    def get_L(self):
        return self.get_channel(AS7265X_W_L_F,AS72652_VISIBLE)
    
    #Get the various NIR readings
    def get_R(self):
        return self.get_channel(AS7265X_R_G_A,AS72651_NIR)
    def get_S(self):
        return self.get_channel(AS7265X_S_H_B,AS72651_NIR)
    def get_T(self):
        return self.get_channel(AS7265X_T_I_C,AS72651_NIR)
    def get_U(self):
        return self.get_channel(AS7265X_U_J_D,AS72651_NIR)
    def get_V(self):
        return self.get_channel(AS7265X_V_K_E,AS72651_NIR)
    def get_W(self):
        return self.get_channel(AS7265X_W_L_F,AS72651_NIR)
    
    #Get the various UV readings
    def get_A(self):
        return self.get_channel(AS7265X_R_G_A,AS72653_UV)
    def get_B(self):
        return self.get_channel(AS7265X_S_H_B,AS72653_UV)
    def get_C(self):
        return self.get_channel(AS7265X_T_I_C,AS72653_UV)
    def get_D(self):
        return self.get_channel(AS7265X_U_J_D,AS72653_UV)
    def get_E(self):
        return self.get_channel(AS7265X_V_K_E,AS72653_UV)
    def get_F(self):
        return self.get_channel(AS7265X_W_L_F,AS72653_UV)

    #A the 16-bit value stored in a given channel registerReturns
    def get_channel(self,channel_register,device):
        self.select_device(device)
        colorData = self.virtual_read_register(channel_register) << 8 #High uint8_t
        colorData = colorData | self.virtual_read_register(channel_register + 1) #Low uint8_t
        return colorData
    
    #Returns the various calibration data
    #UV
    def get_calibrated_A(self):
        return self.get_calibrated_Value(AS7265X_R_G_A_CAL,AS72653_UV)
    def get_calibrated_B(self):
        return self.get_calibrated_Value(AS7265X_S_H_B_CAL,AS72653_UV)
    def get_calibrated_C(self):
        return self.get_calibrated_Value(AS7265X_T_I_C_CAL,AS72653_UV)
    def get_calibrated_D(self):
        return self.get_calibrated_Value(AS7265X_U_J_D_CAL,AS72653_UV)
    def get_calibrated_E(self):
        return self.get_calibrated_Value(AS7265X_V_K_E_CAL,AS72653_UV)
    def get_calibrated_F(self):
        return self.get_calibrated_Value(AS7265X_W_L_F_CAL,AS72653_UV)
    #Color
    def get_calibrated_G(self):
        return self.get_calibrated_Value(AS7265X_R_G_A_CAL,AS72652_VISIBLE)
    def get_calibrated_H(self):
        return self.get_calibrated_Value(AS7265X_S_H_B_CAL,AS72652_VISIBLE)
    def get_calibrated_I(self):
        return self.get_calibrated_Value(AS7265X_T_I_C_CAL,AS72652_VISIBLE)
    def get_calibrated_J(self):
        return self.get_calibrated_Value(AS7265X_U_J_D_CAL,AS72652_VISIBLE)
    def get_calibrated_K(self):
        return self.get_calibrated_Value(AS7265X_V_K_E_CAL,AS72652_VISIBLE)
    def get_calibrated_L(self):
        return self.get_calibrated_Value(AS7265X_W_L_F_CAL,AS72652_VISIBLE)
    #NIR
    def get_calibrated_R(self):
        return self.get_calibrated_Value(AS7265X_R_G_A_CAL,AS72651_NIR)
    def get_calibrated_S(self):
        return self.get_calibrated_Value(AS7265X_S_H_B_CAL,AS72651_NIR)
    def get_calibrated_T(self):
        return self.get_calibrated_Value(AS7265X_T_I_C_CAL,AS72651_NIR)
    def get_calibrated_U(self):
        return self.get_calibrated_Value(AS7265X_U_J_D_CAL,AS72651_NIR)
    def get_calibrated_V(self):
        return self.get_calibrated_Value(AS7265X_V_K_E_CAL,AS72651_NIR)
    def get_calibrated_W(self):
        return self.get_calibrated_Value(AS7265X_W_L_F_CAL,AS72651_NIR)
    
    #Given an address , read four bytes and return the floating point calibrated value
    def get_calibrated_Value(self,calAdress,device):
        self.select_device(device)
        b_arr = bytearray(4)
        b_arr[0] = self.virtual_read_register(calAdress + 0);
        b_arr[1] = self.virtual_read_register(calAdress + 1);
        b_arr[2] = self.virtual_read_register(calAdress + 2);
        b_arr[3] = self.virtual_read_register(calAdress + 3);
        return (struct.unpack('>f',b_arr)[0])
    
    #Mode 0 : 4 channels out of 6 (see datasheet)
    #Mode 1 : Different 4 channels out of 6(see datasheet)
    #Mode 2 : All 6 channels continuously
    #Mode 3 : One-shot reading of all channels
    def set_measurement_mode(self,mode):
        if mode > 0b11 :
            mode = 0b11 #Error check
        #Read , mask/set , write
        value = self.virtual_read_register(AS7265X_CONFIG)
        value = value & 0b11110011
        value = value | (mode << 2) #Set BANK bits with user's choice
        self._mode = mode
        self.virtual_write_register(AS7265X_CONFIG,value)
    
    #Sets the gain value
    #Gain 0: 1x (power-on default)
    #Gain 1: 3.7x
    #Gain 2: 16x
    #Gain 3: 64x
    def set_gain(self,gain):
        if gain > 0b11:
            gain = 0b11
        #Read , mask/set , write
        value = self.virtual_read_register(AS7265X_CONFIG)
        value = value & 0b11001111
        value = value | (gain << 4) #Set GAIN bits with user's choice
        self._gain = gain
        self.virtual_write_register(AS7265X_CONFIG,value)
    
    #Sets the integration cycle amount
    #Give this function a byte from 0 to 255
    #Time will be 2.8ms * [integration cycles + 1]
    def set_integration_cycles(self,cyclevalue):
        if cyclevalue > 255:
            cyclevalue = 255 #Error check
        self.virtual_write_register(AS7265X_INTERGRATION_TIME,cyclevalue)
    
    #To enable module interrupt
    def enable_interrupt(self):
        value = self.virtual_read_register(AS7265X_CONFIG)
        value = value | (1 << 6)
        self.virtual_write_register(AS7265X_CONFIG,value)
    
    #Disable  the interrupt pin
    def disable_interrupt(self):
        value = self.virtual_read_register(AS7265X_CONFIG)
        value = value & ~(1 << 6 )
        self.virtual_write_register(AS7265X_CONFIG,value)

    #Checks to see if DRDY flag is set in the control setup register
    def data_available(self):
        value = self.virtual_read_register(AS7265X_CONFIG)
        return (value & (1 << 1))
    
    #Clear DATA_RDY flag when using mode 3
    def clear_data_available(self):
        value = self.virtual_read_register(AS7265X_CONFIG)
        value = value & ~(1 << 1)#Set the DATA_RDY bit
        self.virtual_write_register(AS7265X_CONFIG,value)

    #Enable the LED or bulb on a given device
    def enable_bulb(self,device):
        self.select_device(device)
        #Read , mask/set , write
        value = self.virtual_read_register(AS7265X_LED_CONFIG)
        value = value | (1 << 3 )
        self.virtual_write_register(AS7265X_LED_CONFIG,value)
    
    #Disable the LED or bulb on a give device
    def disable_bulb(self,device):
        self.select_device(device)
        #Read , mask/set , write
        value = self.virtual_read_register(AS7265X_LED_CONFIG)
        value = value & ~(1 << 3 )
        self.virtual_write_register(AS7265X_LED_CONFIG,value)
    
    #Set the current limit of bulb/LED
    #Current 0: 12.5mA
    #Current 1: 25mA
    #Current 2: 50mA
    #Current 3: 100mA
    def set_bulb_current(self,current,device):
        self.select_device(device)
        #set the current
        if current > 0b11:
            current = 0b11 #limit to two bits
        value = self.virtual_read_register(AS7265X_LED_CONFIG)
        value = value & 0b11001111 #Clear ICL_DRV bits
        value = value | (current << 4) #Set ICL_DRV bits with user's choice
        self.virtual_write_register(AS7265X_LED_CONFIG,value)
    
    #As we read various registers we have to point at the master or first/second slave
    def select_device(self,device):
        #Set the bits 0:1. Just overwrite whatever is there because masking in the correct value doesn't work.
        self.virtual_write_register(AS7265X_DEV_SELECT_CONTROL,device)
    
    #Enable the onboard indicator LED
    def enable_indicator(self):
        #Read , mask/set , write
        value = self.virtual_read_register(AS7265X_LED_CONFIG)
        value = value | (1<<0)#Set the bit
        self.select_device(AS72651_NIR)
        self.virtual_write_register(AS7265X_LED_CONFIG,value)
    
    #Disable the onboard indicator LED
    def disable_indicator(self):
        #Read , mask/set , write
        value = self.virtual_read_register(AS7265X_LED_CONFIG)
        value = value & ~(1<<0)#Set the bit
        self.select_device(AS72651_NIR)
        self.virtual_write_register(AS7265X_LED_CONFIG,value)
    
    #Set the current limit of onboard LED. Default is max 8mA = 0b11.
    #Current 0 : 1 mA
    #Current 1 : 2 mA
    #Current 2 : 4 mA
    #Current 3 : 8 mA
    def set_indicator_current(self,current):
        if current > 0b11:
            current = 0b11
        value = self.virtual_read_register(AS7265X_LED_CONFIG)
        value = value & 0b11111001 #Clear ICL_IND bits
        value = value | (current << 1 ) #Set ICL_IND bits with user's choice
        self.select_device(AS72651_NIR)
        self.virtual_write_register(AS7265X_LED_CONFIG,value)
    
    #Returns the temperature of a given device in C
    def get_temperature(self,devicenumber):
        self.select_device(devicenumber)
        return self.virtual_read_register(AS7265X_DEVICE_TEMP)
    
    #Returns an average of all the sensor temps in C
    def get_temperature_average(self):
        average = 0
        for i in range(0,3):
            average = average + getTemperature(i)
        return average/3
    
    #Does a soft reset
    #Give sensor at least 1000ms to reset
    def soft_reset(self):
        #Read , mask/set , write
        value = self.virtual_read_register(AS7265X_CONFIG)
        value = value | (1 << 7 )#Set RST bit , automatically cleared after reset
        self.virtual_write_register(AS7265X_CONFIG,value)

    #Get all values
    def get_value(self,RorC): 
        #RorC is 0 read Raw data
        #RorC is 1 read Calibrated data
        value = []
        if self._mode == AS7265X_MEASUREMENT_MODE_4CHAN:
            #print color sequence
            print("S / T / U / V / I / G / H / K / C / A / B / E \r\n")
            if RorC == 0:
                value.append(self.get_S())
                value.append(self.get_T())
                value.append(self.get_U())
                value.append(self.get_V())
                value.append(self.get_I())
                value.append(self.get_G())
                value.append(self.get_H())
                value.append(self.get_K())
                value.append(self.get_C())
                value.append(self.get_A())
                value.append(self.get_B())
                value.append(self.get_E())
            else:
                value.append(self.get_calibrated_S())
                value.append(self.get_calibrated_T())
                value.append(self.get_calibrated_U())
                value.append(self.get_calibrated_V())
                value.append(self.get_calibrated_I())
                value.append(self.get_calibrated_G())
                value.append(self.get_calibrated_H())
                value.append(self.get_calibrated_K())
                value.append(self.get_calibrated_C())
                value.append(self.get_calibrated_A())
                value.append(self.get_calibrated_B())
                value.append(self.get_calibrated_E())
        elif self._mode == AS7265X_MEASUREMENT_MODE_4CHAN_2:
            #print color sequence
            print("R / T / U / W / L / G / H / J / F / A / B / D \r\n")
            if RorC == 0:
                value.append(self.get_R())
                value.append(self.get_T())
                value.append(self.get_U())
                value.append(self.get_W())
                value.append(self.get_L())
                value.append(self.get_G())
                value.append(self.get_H())
                value.append(self.get_J())
                value.append(self.get_F())
                value.append(self.get_A())
                value.append(self.get_B())
                value.append(self.get_D())
            else:
                value.append(self.get_calibrated_R())
                value.append(self.get_calibrated_T())
                value.append(self.get_calibrated_U())
                value.append(self.get_calibrated_W())
                value.append(self.get_calibrated_L())
                value.append(self.get_calibrated_G())
                value.append(self.get_calibrated_H())
                value.append(self.get_calibrated_J())
                value.append(self.get_calibrated_F())
                value.append(self.get_calibrated_A())
                value.append(self.get_calibrated_B())
                value.append(self.get_calibrated_D())
        elif self._mode == AS7265X_MEASUREMENT_MODE_6CHAN_CONTINUOUS or self._mode == AS7265X_MEASUREMENT_MODE_6CHAN_ONE_SHOT:
            #print color sequence
            print("R / S / T / U / V / W / G / H / I / J / K / L / A / B / C / D / E / F \r\n")
            if RorC == 0:
                #NIR
                value.append(self.get_R())
                value.append(self.get_S())
                value.append(self.get_T())
                value.append(self.get_U())
                value.append(self.get_V())
                value.append(self.get_W())
                #Color
                value.append(self.get_G())
                value.append(self.get_H())
                value.append(self.get_I())
                value.append(self.get_J())
                value.append(self.get_K())
                value.append(self.get_L())
                #UV
                value.append(self.get_A())
                value.append(self.get_B())
                value.append(self.get_C())
                value.append(self.get_D())
                value.append(self.get_E())
                value.append(self.get_F())
            else:
                #NIR
                value.append(self.get_calibrated_R())
                value.append(self.get_calibrated_S())
                value.append(self.get_calibrated_T())
                value.append(self.get_calibrated_U())
                value.append(self.get_calibrated_V())
                value.append(self.get_calibrated_W())
                #Color
                value.append(self.get_calibrated_G())
                value.append(self.get_calibrated_H())
                value.append(self.get_calibrated_I())
                value.append(self.get_calibrated_J())
                value.append(self.get_calibrated_K())
                value.append(self.get_calibrated_L())
                #UV
                value.append(self.get_calibrated_A())
                value.append(self.get_calibrated_B())
                value.append(self.get_calibrated_C())
                value.append(self.get_calibrated_D())
                value.append(self.get_calibrated_E())
                value.append(self.get_calibrated_F())
        return value