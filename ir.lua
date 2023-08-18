function sysCall_threadmain()

-- Get FL Steering Motor.
  FLwheel_Steering = sim.getObjectHandle('MyBot_FLwheel_Steering')
-- Get FR Steering Motor.
  FRwheel_Steering = sim.getObjectHandle('MyBot_FRwheel_Steering')
-- Get FL Motor.
  FLwheel_Motor = sim.getObjectHandle('MyBot_FLwheel_Motor')
-- Get FR Motor.
  FRwheel_Motor = sim.getObjectHandle('MyBot_FRwheel_Motor')
-- Get Vision Sensors
  Vision_Sensors = {1, 1, 1}
-- Get Left Vision Sensor.
  Vision_Sensors[1] = sim.getObjectHandle('MyBot_Left_Sensor')
-- Get Middle Vision Sensor.
  Vision_Sensors[2] = sim.getObjectHandle('MyBot_Middle_Sensor')
-- Get Right Vision Sensor.
  Vision_Sensors[3] = sim.getObjectHandle('MyBot_Right_Sensor')
-- d = 0.2 * distance (cm) between left and right wheels.
-- l = 0.2 * distance (cm) between front and rear wheels.
d = 28 * 0.2
l = 30 * 0.2
-- Angle to steer.
-- +: left
-- -: right
steeringAngle = 0
-- It's used to control the total time of avoiding.
avoidUntilTime = 0
-- PID Control
integral = 0
derivative = 0
prev_error = 0

-- Steer Tool Function
  steer = function(angle)
       -- Conversion
       angle = angle * math.pi / 180
       leftAngle = math.atan(l / (-d + l / math.tan(angle)))
       rightAngle = math.atan(l / (d + l / math.tan(angle)))
       sim.setJointTargetPosition(FLwheel_Steering, leftAngle)
       sim.setJointTargetPosition(FRwheel_Steering, rightAngle)
  end

-- Motor Tool Function
   motor = function(speed) sim.setJointTargetVelocity(FLwheel_Motor, speed)
sim.setJointTargetVelocity(FRwheel_Motor, speed)
end

-- The Main Loop
while sim.getSimulationState() ~= sim.simulation_advancing_abouttostop
do
results = {-1, -1, -1}
for i = 1, 3, 1 do
result, data = sim.readVisionSensor(Vision_Sensors[i])
if (result >= 0) then
results[i] = data[11]
end
end

-- P Control
kp = 50
error = (results[3] - results[1])
-- I Control
ki = 0.01
integral = integral + error
-- D Control
kd = 1
derivative = error - prev_error prev_error = error
-- PID Control
steeringAngle = kp * error + ki * integral + kd * derivative
-- Steer for calculated angle.
steer(steeringAngle)
-- Motor Speed.
motor(4)


-- -- Since this script is threaded, don't waste time here.
sim.switchThread()
end
end
