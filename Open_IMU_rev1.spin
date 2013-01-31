'' Open_IMU
'' based on Madgwick Algorithm for MARG - uses gradient descent algo.

CON

  _clkmode      = xtal1 + pll16x
  _clkfreq      = 80_000_000

  SDA = 6
  SCL = 7

  COMPASS_XMIN = -345.0
  COMPASS_XMAX =  216.0
  COMPASS_YMIN = -347.0
  COMPASS_YMAX =  210.0
  COMPASS_ZMIN = -305.0
  COMPASS_ZMAX =  249.0

  INVERSE_XRANGE = 2.0 / (COMPASS_XMAX - COMPASS_XMIN)
  INVERSE_YRANGE = 2.0 / (COMPASS_YMAX - COMPASS_YMIN) 
  INVERSE_ZRANGE = 2.0 / (COMPASS_ZMAX - COMPASS_ZMIN)

  SAMPLE_FREQ = 512.0
  TWO_KP_DEF = 2.0 * 0.5
  TWO_KI_DEF = 2.0 * 0.00
  BETA_DEF = 0.08

  GYRO_SCALE = 0.07
                         
  PIBY2_FLOAT = 1.5707963
  PI_FLOAT = 3.14159265

  MILLI = _clkfreq / 1_000


VAR

  long rawAcclX, rawAcclY, rawAcclZ
  long rawMagX, rawMagY, rawMagZ
  long rawGyroX, rawGyroY, rawGyroZ
  
  long q0, q1, q2, q3, beta

  long magnitude

  long pitch, roll, yaw

  long gyroSumX, gyroSumY, gyroSumZ
  long offSetX, offSetY, offSetZ

  long floatMagX, floatMagY, floatMagZ
  long smoothAccX, smoothAccY, smoothAccZ
  long accToFilterX, accToFilterY, accToFilterZ
  
  
OBJ

  IMU         : "MinIMUv2-pasm"                '
  PST         : "Parallax Serial Terminal"
  fm          : "Float32Full"
  fs          : "FloatString"
    
PUB Init

  IMU.start(SCL,SDA)
  fm.start
  PST.start(250000)

  Main

PUB Main


PRI IMUinit

    beta := BetaDef

    repeat 500
        rawGyroX := fm.FFloat(IMU.getRx)
        rawGyroY := fm.FFloat(IMU.getRy)
        rawGyroZ := fm.FFloat(IMU.getRz)

        rawAcclX := fm.FFloat(IMU.getAx)
        rawAcclY := fm.FFloat(IMU.getAy)
        rawAcclZ := fm.FFloat(IMU.getAz)

        rawMagX := fm.FFloat(IMU.getMx)
        rawMagY := fm.FFloat(IMU.getMy)
        rawMagZ := fm.FFloat(IMU.getMz)

        SmoothAccX := Smoothing(rawAcclX, rawMagX)
        SmoothAccY := Smoothing(rawAcclY, rawMagY)
        SmoothAccZ := Smoothing(rawAcclZ, rawMagZ)

        waitcnt(clkfreq/1_000 * 3 + cnt)

    gyroSumX := gyroSumY := gyroSumZ := 0

    repeat 500
        'This rawGyro group is NOT floating, since we are doing integer math below for the average.
        rawGyroX := IMU.getRx
        rawGyroY := IMU.getRy
        rawGyroZ := IMU.getRz 

        rawAcclX := fm.FFloat(IMU.getAx)
        rawAcclY := fm.FFloat(IMU.getAy)
        rawAcclZ := fm.FFloat(IMU.getAz)     

        rawMagX := fm.FFloat(IMU.getMx)
        rawMagY := fm.FFloat(IMU.getMy)
        rawMagZ := fm.FFloat(IMU.getMz)   

        SmoothAccX := Smoothing(rawAcclX, rawMagX)
        SmoothAccY := Smoothing(rawAcclY, rawMagY)
        SmoothAccZ := Smoothing(rawAcclZ, rawMagZ)

        gyroSumX += rawGyroX
        gyroSumY += rawGyroY
        gyroSumZ += rawGyroZ

        waitcnt(clkfreq/1_000 * 3 + cnt)

    offSetX := fm.FDiv(fm.FFloat(gyroSumX), 500.0)
    offSetY := fm.FDiv(fm.FFloat(gyroSumY), 500.0)
    offSetZ := fm.FDiv(fm.FFloat(gyroSumZ), 500.0)

    'Calculate initial Quaternion
    'pitch := ToDeg(fastAtan2(rawAcclX, sqrt(rawAcclY * rawAcclY + rawAcclZ * rawAcclZ)))
    'roll := ToDeg(fastAtan2(-1 * rawAcclY, sqrt(rawAcclX * rawAcclX + rawAcclZ * rawAcclZ)))

    AcclXX := fm.FMul(rawAcclX, rawAcclX)
    AcclYY := fm.FMul(rawAcclY, rawAcclY)
    AcclZZ := fm.FMul(rawAcclZ, rawAcclZ)
    negAcclY := fm.FNeg(rawAcclY)
    
    l := fm.FAdd(AcclYY, AcclZZ)
    m := fm.FSqr(l)
    n := fastAtan2(rawAcclX, m)
    pitch := ToDeg(n)
    
    l := fm.FAdd(AcclXX, AcclZZ)
    m := fm.FSqr(l)
    n := fastAtan2(negAcclY, m)
    roll := ToDeg(n)
    
    if rawAcclZ > 0.0
        if rawAcclX > 0.0
            pitch := fm.FSub(180.0, pitch)
        else
            pitch = fm.FSub(-180.0, pitch)
        if rawAcclY > 0.0
            roll := fm.FSub(-180.0, roll)
        else
            roll := fm.FSub(180.0, roll)
    
    floatMagX = (rawMagX - compassXMin) * inverseXRange - 1.0
    floatMagY = (rawMagY - compassYMin) * inverseYRange - 1.0
    floatMagZ = (rawMagZ - compassZMin) * inverseZRange - 1.0
    
    sin_pitch := fm.Sin(ToRad(pitch))
    cos_pitch := fm.Cos(ToRad(pitch))
    sin_roll := fm.Sin(ToRad(roll))
    cos_roll := fm.Cos(ToRad(roll))
    
    'xMag := (floatMagX * cos(ToRad(pitch))) + floatMagZ * sin(ToRad(pitch)))
    j := fm.FMul(cos_pitch, floatMagX)
    k := fm.FMul(sin_pitch, floatMagZ)
    xMag := fm.FAdd(j, k)
    
    
    'yMag := -1 * ((floatMagX * sin(ToRad(roll)) * sin(ToRad(pitch))) + (floatMagY * cos(ToRad(roll))) - (floatMagZ * sin(ToRad(roll)) * cos(ToRad(pitch))))
    j := fm.FMul(fm.FMul(floatMagX, sin_roll), sin_pitch)
    negJ := fm.FNeg(j)
    k := fm.FMul(floatMagY, cos_roll)
    l := fm.FMul(fm.FMul(floatMagZ, sin_roll), cos_pitch)
    yMag := fm.FSub(fm.FAdd(negJ, k), l)
    
    yaw := ToDeg(fastAtan2(yMag, xMag))
    
    if yaw < 0.0
        yaw := fm.FAdd(yaw, 360.0)
    
    cosPitch := fm.Cos(ToRad(pitch))
    sinPitch := fm.Sin(ToRad(pitch))
    cosRoll := fm.Cos(ToRad(roll))
    sinRoll := fm.Sin(ToRad(roll))
    cosYaw := fm.Cos(ToRad(Yaw))
    sinYaw := fm.Sin(ToRad(Yaw))
        
    r11 := fm.FMul(cosPitch, cosYaw)
    r21 := fm.FMul(cosPitch, sinYaw)
    r31 := fm.FNeg(sinPitch)
    
    r12 := fm.FAdd(fm.FNeg(fm.FMul(cosRoll, sinYaw)), fm.FMul(fm.FMul(sinRoll, sinPitch), cosYaw))
    r22 := fm.FAdd(fm.FMul(cosRoll, cosYaw), fm.FMul(fm.FMul(sinRoll, sinPitch), sinYaw))
    r23 := fm.FMul(sinRoll, cosPitch)
    
    r31 := fm.FAdd(fm.FMul(sinRoll, sinYaw), fm.FMul(fm.FMul(cosRoll, sinPitch), cosYaw))
    r32 := fm.FAdd(fm.FNeg(fm.FMul(sinRoll, cosYaw)), fm.FMul(fm.FMul(cosRoll, sinPitch), sinYaw))
    r33 := fm.FMul(cosRoll, cosPitch)
    
    q_denom := fm.FMul(4.0, q0)
    
    q0 := fm.FDiv(0.5, fm.FSqr(fm.FAdd(fm.FAdd(1.0, r11), fm.FAdd(r22, r33))))
    q1 := fm.FDiv(fm.FSub(r32, r23), q_denom)
    q2 := fm.FDiv(fm.FSub(r13, r31), q_denom)
    q3 := fm.FDiv(fm.FSub(r21, r12), q_denom)
    
PRI IMUupdate | gx, gy, gz, ax, ay, az, recipNorm, s0, s1, s2, s3, qDot1, qDot2, qDot3, qDot4, _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 ,_8q1, _8q2, q0q0, q1q1, q2q2, q3q3 

PRI AHRSupdate | gx, gy, gz, ax, ay, az, mx, my, mz, recipNorm, s0, s1, s2, s3, qDot1, qDot2, qDot3, qDot4, _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 ,_8q1, _8q2, q0q0, q1q1, q2q2, q3q3

PRI GetEuler

PRI FastAtan2(y, x) | atan, z

    if x == 0.0
        if y > 0.0
            return PIBY2_FLOAT
        elif y == 0.0
            return 0.0
        else
            return fm.FNeg(PIBY2_FLOAT)
            
    z := fm.FDiv(y, x)
    
    if fm.FAbs(z) < 1.0
        j := fm.FMul(0.28, z)
        k := fm.FMul(j, z)
        l := fm.FAdd(1.0, k)
        
        atan := fm.FDiv(z, l)
        
        if x < 0.0
            if y < 0.0
                return(fm.FSub(atan, PI_FLOAT))
            else
                return(fm.FAdd(atan, PI_FLOAT))

PRI invSqrt(number) | i, x, y

    result := fm.Pow(x, FNeg(0.5))
    
PRI Smoothing(raw, smooth)

    result := fm.FAdd(fm.FMul(raw, 0.15), FM.FMul(smooth * 0.85))

PRI ToDeg(x) | i
    
    result := fm.FMul(x, 57.2957795131)

PRI ToRad(x)

    result := fm.FMul(x, 0.01745329252)

    



