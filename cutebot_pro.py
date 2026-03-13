from microbit import i2c, sleep
import time
# I2C address for Cutebot Pro
I2C_ADDR = 0x10

# Enums for Cutebot Pro components
class CutebotProMotors:
    M1 = 1  # Left wheel
    M2 = 2  # Right wheel
    ALL = 3  # All wheels

class CutebotProRGBLight:
    RGBL = 2  # Left RGB
    RGBR = 1  # Right RGB
    RGBA = 3  # All RGB lights

class CutebotProSpeedUnits:
    Cms = 0  # cm/s
    Ins = 1  # inch/s

class SonarUnit:
    Centimeters = 0
    Inches = 1


class CutebotPro:
    def __init__(self):
        """Initialize Cutebot Pro"""
        self.version = -1
        self.IR_Val = 0
        self._detect_hardware_version()
    
    # set reg
    def __sr(self, reg, dat):
        i2c.write(I2C_ADDR, bytearray([reg, dat]))
    
    # get reg
    def __gr(self, reg):
        i2c.write(I2C_ADDR, bytearray([reg]))
        t = i2c.read(I2C_ADDR, 1)
        return t[0]
    
    # read block
    def __read_block(self, reg, length):
        i2c.write(I2C_ADDR, bytearray([reg]))
        t = i2c.read(I2C_ADDR, length)
        return t
    
    def _detect_hardware_version(self):
        """Detect hardware version of Cutebot Pro"""
        try:
            # Create buffer for version detection
            buffer = [0x99, 0x15, 0x01, 0x00, 0x00, 0x00, 0x88]
            i2c.write(I2C_ADDR, bytearray(buffer))
            sleep(0.05)
            version = self.__gr(0x00)
            if version == 1:
                self.version = 1
            else:
                self.version = 2
        except Exception as e:
            print("Error detecting hardware version: {}".format(e))
            self.version = 1  # Default to version 1 if detection fails
    
    def get_hardware_version(self):
        """Return detected hardware version"""
        return self.version
    
    def _i2c_write_buffer(self, buffer):
        """Write buffer to I2C device"""
        try:
            i2c.write(I2C_ADDR, bytearray(buffer))
            sleep(0.001)  # Small delay after I2C write
        except Exception as e:
            print("I2C write error: {}".format(e))
    
    def _i2c_command_send_v2(self, command, params):
        """Send I2C command for V2 hardware"""
        buff = [0xFF, 0xF9, command, len(params)] + params
        self._i2c_write_buffer(buff)
    
    def fullSpeedAhead(self):
        """Go forward at full speed"""
        if self.version == 2:
            # V2 implementation: use motorControl with 100% speed
            self._i2c_command_send_v2(0x10, [2, 100, 100, 0])  # 2=all wheels, 100=speed, 0=forward direction
        else:
            # V1 implementation
            buffer = [0x99, 0x07, 0x00, 0x00, 0x00, 0x00, 0x88]
            self._i2c_write_buffer(buffer)
    
    def fullAstern(self):
        """Go reverse at full speed"""
        if self.version == 2:
            # V2 implementation: use motorControl with -100% speed
            self._i2c_command_send_v2(0x10, [2, 100, 100, 0x03])  # 2=all wheels, 100=speed, 0x03=both reverse
        else:
            # V1 implementation
            buffer = [0x99, 0x08, 0x00, 0x00, 0x00, 0x00, 0x88]
            self._i2c_write_buffer(buffer)
    
    def pwmCruiseControl(self, speedL, speedR):
        """PWM control the car to travel at a specific speed"""
        if self.version == 2:
            # V2 implementation
            direction = 0
            if speedL < 0:
                direction |= 0x01
            if speedR < 0:
                direction |= 0x02
            self._i2c_command_send_v2(0x10, [2, abs(speedL), abs(speedR), direction])
        else:
            # V1 implementation
            # Handle left wheel
            if speedL == 0:
                speedL = 200
            elif speedL > 0:
                # Map speed from 0-100 to 20-100
                speedL = int((speedL / 100) * 80 + 20)
            else:
                # Map speed from -100-0 to -100--20
                speedL = -int((abs(speedL) / 100) * 80 + 20)
            
            # Handle right wheel
            if speedR == 0:
                speedR = 200
            elif speedR > 0:
                # Map speed from 0-100 to 20-100
                speedR = int((speedR / 100) * 80 + 20)
            else:
                # Map speed from -100-0 to -100--20
                speedR = -int((abs(speedR) / 100) * 80 + 20)
            
            # Send left wheel command
            if speedL > 0:
                buffer = [0x99, 0x01, 1, 0x01, speedL, 0x00, 0x88]
            else:
                buffer = [0x99, 0x01, 1, 0x00, -speedL, 0x00, 0x88]
            self._i2c_write_buffer(buffer)
            
            # Send right wheel command
            if speedR > 0:
                buffer = [0x99, 0x01, 2, 0x01, speedR, 0x00, 0x88]
            else:
                buffer = [0x99, 0x01, 2, 0x00, -speedR, 0x00, 0x88]
            self._i2c_write_buffer(buffer)
  
    def stopImmediately(self, wheel):
        """Stop immediately"""
        if self.version == 2:
            # V2 implementation: wheel 0=left, 1=right, 2=all
            wheel_map = {1: 0, 2: 1, 3: 2}  # Map CutebotProMotors to V2 wheel codes
            mapped_wheel = wheel_map.get(wheel, 2)
            self._i2c_command_send_v2(0x10, [mapped_wheel, 0, 0, 0])
        else:
            # V1 implementation
            buffer = [0x99, 0x09, wheel, 0x00, 0x00, 0x00, 0x88]
            self._i2c_write_buffer(buffer)
    
    def readSpeed(self, motor, speedUnits):
        """Read motor speed"""
        if self.version == 2:
            # V2 implementation
            self._i2c_command_send_v2(0xA0, [motor])  # motor 0=M1, 1=M2
            speed = self.__gr(0x00)
        else:
            # V1 implementation
            buffer = [0x99, 0x05, motor, 0x00, 0x00, 0x00, 0x88]
            self._i2c_write_buffer(buffer)
            speed = self.__gr(0x00)
        
        if speedUnits == CutebotProSpeedUnits.Cms:
            return speed
        else:
            return speed / 0.3937
    
    def _pulseNumber(self):
        """Get encoder motor pulse counts"""
        if self.version == 1:
            buffer = [0x99, 0x16, 0x00, 0x00, 0x00, 0x00, 0x88]
            self._i2c_write_buffer(buffer)
            
            # Read 10 bytes: 4 for left pulse, 4 for right pulse, 1 for left direction, 1 for right direction
            pulse_data = []
            for _ in range(10):
                pulse_data.append(self.__gr(0x00))
            
            # Calculate left pulse count
            pulseCntL = (pulse_data[0] << 24) | (pulse_data[1] << 16) | (pulse_data[2] << 8) | pulse_data[3]
            # Calculate right pulse count
            pulseCntR = (pulse_data[4] << 24) | (pulse_data[5] << 16) | (pulse_data[6] << 8) | pulse_data[7]
            
            # Apply direction
            if pulse_data[8] == 1:
                pulseCntL = -pulseCntL
            if pulse_data[9] == 1:
                pulseCntR = -pulseCntR
            
            return pulseCntL, pulseCntR
        return 0, 0
    
    def readDistance(self, motor):
        """Get rotation degrees of wheel"""
        if self.version == 2:
            # V2 implementation
            # motor: M1=1, M2=2 -> param: 3, 4 (motor + 2)
            motor_param = motor + 2
            print("[DEBUG] readDistance V2, motor: {}, param: {}".format(motor, motor_param))
            self._i2c_command_send_v2(0xA0, [motor_param])
            sleep(0.001)
            data = self.__read_block(0x00, 4)
            print("[DEBUG] raw data: {}, {}, {}, {}".format(data[0], data[1], data[2], data[3]))
            distance = (data[0] | (data[1] << 8) | (data[2] << 16) | (data[3] << 24))
            if distance & 0x80000000:
                distance -= 0x100000000
            print("[DEBUG] final distance: {}".format(distance))
            return distance
        else:
            # V1 implementation
            pulseCntL, pulseCntR = self._pulseNumber()
            if motor == CutebotProMotors.M1:
                return int(pulseCntL * 360 / 1428 + 0.5)
            else:
                return int(pulseCntR * 360 / 1428 + 0.5)
    
    def clearWheelTurn(self, motor):
        """Clear rotation degrees of wheel"""
        if self.version == 2:
            # V2 implementation: motor 0=M1, 1=M2
            # motor: M1=1, M2=2 -> param: 3, 4 (motor + 2)
            motor_param = motor + 2
            print("[DEBUG] clearWheelTurn V2, motor: {}, param: {}".format(motor, motor_param))
            self._i2c_command_send_v2(0x50, [motor_param])
            print("[DEBUG] sent clear command 0x50, param: {}".format(motor_param))
        else:
            # V1 implementation
            buffer = [0x99, 0x0A, motor, 0x00, 0x00, 0x00, 0x88]
            self._i2c_write_buffer(buffer)
   
    def ultrasonic(self, unit, maxCmDistance=500):
        """Read ultrasonic sensor distance"""
        # Note: This function uses GPIO pins, not I2C
        # Implementation for micro:bit
        print("[DEBUG] ultrasonic start")
        
        # Read multiple times and filter
        readings = []
        for attempt in range(3):
            distance = self._ultrasonic_read_single(maxCmDistance)
            if distance > 0 and distance < maxCmDistance:
                readings.append(distance)
            sleep(0.05)  # Wait between readings
        
        if len(readings) == 0:
            print("[DEBUG] no valid readings")
            return 0
        
        # Return median value
        readings.sort()
        result = readings[len(readings) // 2]
        print("[DEBUG] final distance: {}".format(result))
        
        if unit == SonarUnit.Centimeters:
            return result
        else:
            return int(result * 0.3937)
    
    def _ultrasonic_read_single(self, maxCmDistance=500):
        """Single ultrasonic read"""
        from microbit import pin8, pin12
        
        TRIG = pin8
        ECHO = pin12
        
        # Send trigger pulse
        TRIG.write_digital(0)
        sleep(0.005)
        TRIG.write_digital(1)
        sleep(0.01)
        TRIG.write_digital(0)
        
        # Wait for echo start
        timeout = time.ticks_add(time.ticks_ms(), 300)
        while ECHO.read_digital() == 0:
            if time.ticks_diff(timeout, time.ticks_ms()) < 0:
                return 0
        
        pulse_start = time.ticks_ms()
        
        # Wait for echo end
        timeout = time.ticks_add(time.ticks_ms(), 300)
        while ECHO.read_digital() == 1:
            if time.ticks_diff(timeout, time.ticks_ms()) < 0:
                return 0
        
        pulse_end = time.ticks_ms()
        
        # Calculate distance
        pulse_duration = time.ticks_diff(pulse_end, pulse_start) / 1000
        distance_cm = pulse_duration * 17150
        
        return int(distance_cm)
    
    def singleHeadlights(self, light, r, g, b):
        """Select a headlights and set the RGB color"""
        if self.version == 2:
            # V2 implementation: light 0=left, 1=right, 2=all
            light_map = {1: 1, 2: 0, 3: 2}  # Map CutebotProRGBLight to V2 light codes
            mapped_light = light_map.get(light, 2)
            self._i2c_command_send_v2(0x20, [mapped_light, abs(r), abs(g), abs(b)])
        else:
            # V1 implementation
            buffer = [0x99, 0x0F, 0x00, r, g, b, 0x88]
            if light == 3:
                buffer[2] = 0x03
            elif light == 1:
                buffer[2] = 0x01
            elif light == 2:
                buffer[2] = 0x02
            self._i2c_write_buffer(buffer)
    
    def colorLight(self, light, color):
        """Set LED headlights to a specific color"""
        # Extract RGB from color number
        r = (color >> 16) & 0xFF
        g = (color >> 8) & 0xFF
        b = color & 0xFF
        
        if self.version == 2:
            # V2 implementation: light 0=left, 1=right, 2=all
            light_map = {1: 1, 2: 0, 3: 2}  # Map CutebotProRGBLight to V2 light codes
            mapped_light = light_map.get(light, 2)
            self._i2c_command_send_v2(0x20, [mapped_light, abs(r), abs(g), abs(b)])
        else:
            # V1 implementation
            buffer = [0x99, 0x0F, light, r, g, b, 0x88]
            self._i2c_write_buffer(buffer)
    
    def turnOffAllHeadlights(self):
        """Turn off all the LED lights"""
        if self.version == 2:
            # V2 implementation: use 0x20 command with all lights and 0,0,0
            self._i2c_command_send_v2(0x20, [2, 0, 0, 0])
        else:
            # V1 implementation
            buffer = [0x99, 0x10, 0x03, 0x00, 0x00, 0x00, 0x88]
            self._i2c_write_buffer(buffer)
    
    def trackbitStateValue(self):
        """Get a status value of the 4-way line following sensor"""
        if self.version == 2:
            # V2 implementation
            self._i2c_command_send_v2(0x60, [0x00])
            sleep(0.001)
            self.fourWayStateValue = self.__gr(0x00)
        else:
            # V1 implementation
            buffer = [0x99, 0x12, 0x00, 0x00, 0x00, 0x00, 0x88]
            self._i2c_write_buffer(buffer)
            sleep(0.001)
            self.fourWayStateValue = self.__gr(0x00)
    
    def getOffset(self):
        """4-way line following sensor offset"""
        if self.version == 2:
            # V2 implementation
            self._i2c_command_send_v2(0x60, [0x01])
            sleep(0.001)
            data = self.__read_block(0x00, 2)
            offset = (data[0] << 8) | data[1]
        else:
            # V1 implementation
            buffer = [0x99, 0x14, 0x00, 0x00, 0x00, 0x00, 0x88]
            self._i2c_write_buffer(buffer)
            sleep(0.001)
            offsetLow = self.__gr(0x00)
            
            buffer = [0x99, 0x14, 0x01, 0x00, 0x00, 0x00, 0x88]
            self._i2c_write_buffer(buffer)
            sleep(0.001)
            offsetHigh = self.__gr(0x00)
            
            offset = (offsetHigh << 8) | offsetLow
        
        # Map offset from 0-6000 to -3000-3000
        offset = (offset - 3000)  # This maps 0-6000 to -3000-3000
        return offset
    

   

    

    

    
    def readVersions(self):
        """Read version number"""
        if self.version == 2:
            # V2 implementation
            self._i2c_command_send_v2(0xA0, [0x00])
            sleep(0.001)
            version = self.__read_block(0x00, 3)
            return "V {}.{}.{}".format(version[0], version[1], version[2])
        else:
            # V1 implementation
            buffer = [0x99, 0x15, 0x00, 0x00, 0x00, 0x00, 0x88]
            self._i2c_write_buffer(buffer)
            sleep(0.001)
            cutebotProVersionsDecimal = self.__gr(0x00)
            
            buffer = [0x99, 0x15, 0x01, 0x00, 0x00, 0x00, 0x88]
            self._i2c_write_buffer(buffer)
            sleep(0.001)
            cutebotProVersionsInteger = self.__gr(0x00)
            
            if cutebotProVersionsDecimal / 10 > 1:
                minor_version = cutebotProVersionsDecimal // 10
                patch_version = cutebotProVersionsDecimal % 10
                return "V{}.{}.{}".format(cutebotProVersionsInteger, minor_version, patch_version)
            else:
                patch_version = cutebotProVersionsDecimal % 10
                return "V{}.0.{}".format(cutebotProVersionsInteger, patch_version)
    

