-- This script is threaded! It is a very simple example of how Ackermann steering can be handled.
-- Normally, one would use a non-threaded script for that

function sysCall_threadmain()
    -- Put some initialization code here:
    -- Retrieving of some handles and setting of some initial values:

	-- Retrieving motor
    motorLeft  = sim.getObjectHandle('nakedCar_motorLeft')
    motorRight = sim.getObjectHandle('nakedCar_motorRight')

	-- Retrieving bump sensors
    bumpf = sim.getObjectHandle('Bump_front')
    bumpb = sim.getObjectHandle('Bump_back')

	-- Retrieving Vision Sensors
    visionL1_s = sim.getObjectHandle('Vision01')
    visionL2_s = sim.getObjectHandle('Vision02')
    visionL3_s = sim.getObjectHandle('Vision03')
    visionL4_s = sim.getObjectHandle('Vision04')
    visionL5_s = sim.getObjectHandle('Vision05')
	visionL6_s = sim.getObjectHandle('Vision06')
    visionL7_s = sim.getObjectHandle('Vision07')
    visionC1_s = sim.getObjectHandle('Vision08')
    visionC2_s = sim.getObjectHandle('Vision09')
    visionR1_s = sim.getObjectHandle('Vision10')
    visionR2_s = sim.getObjectHandle('Vision11')
    visionR3_s = sim.getObjectHandle('Vision12')
    visionR4_s = sim.getObjectHandle('Vision13')
	visionR5_s = sim.getObjectHandle('Vision14')
	visionR6_s = sim.getObjectHandle('Vision15')
	visionR7_s = sim.getObjectHandle('Vision16')


	-- Set the base speed of the motor
    desiredLeftBaseWheelRotSpeed = 1
    desiredRightBaseWheelRotSpeed = 1

	-- Set the initial speed of the motor
    desiredLeftWheelRotSpeed = 1
    desiredRightWheelRotSpeed = 1

    --Setting factors
	Kp = 0.9
	Kd = 0.1
	lastError = 0

    -- Driving mode; 1 refers to forward and 2 refers to backward
    drive = 1

    -- Main routine:
	-- Start to execute the loop statement
    while sim.getSimulationState()~=sim.simulation_advancing_abouttostop do

		-- Get the return value of the vision sensor
        L_1, data_L1 = sim.readVisionSensor(visionL1_s)
        L_2, data_L2 = sim.readVisionSensor(visionL2_s)
        L_3, data_L3 = sim.readVisionSensor(visionL3_s)
        L_4, data_L4 = sim.readVisionSensor(visionL4_s)
        L_5, data_L5 = sim.readVisionSensor(visionL5_s)
        L_6, data_L6 = sim.readVisionSensor(visionL6_s)
        L_7, data_L7 = sim.readVisionSensor(visionL7_s)
        C_1, data_C1 = sim.readVisionSensor(visionC1_s)
        C_2, data_C2 = sim.readVisionSensor(visionC2_s)
        R_1, data_R1 = sim.readVisionSensor(visionR1_s)
        R_2, data_R2 = sim.readVisionSensor(visionR2_s)
        R_3, data_R3 = sim.readVisionSensor(visionR3_s)
        R_4, data_R4 = sim.readVisionSensor(visionR4_s)
        R_5, data_R5 = sim.readVisionSensor(visionR5_s)
        R_6, data_R6 = sim.readVisionSensor(visionR6_s)
        R_7, data_R7 = sim.readVisionSensor(visionR7_s)

		-- Detect whether the bump sensor collides
        if(sim.getJointPosition(bumpf) > 0.001) then
			-- The front sensor detects the collision and retreats
            drive = 2
			-- The set back time should be 3 seconds
            backUntilTime = sim.getSimulationTime() + 3
        elseif(sim.getJointPosition(bumpb) < -0.001) then
			-- The rear sensor detects the collision and moves forward
            drive = 1
            desiredLeftWheelRotSpeed = 3
            desiredRightWheelRotSpeed = 3
        end

        --the state of IR sensor, if it is on black it is 0, on white is 1
		if(L_1 >= 0) then
               if(data_L1[11] < 0.3)then
					L1 = 0
				else
					L1 = 1
               end
		end
		if(L_2 >= 0) then
               if(data_L2[11] < 0.3)then
					L2 = 0
				else
					L2 = 1
               end
		end
		if(L_3 >= 0) then
               if(data_L3[11] < 0.3)then
					L3 = 0
				else
					L3 = 1
               end
		end
		if(L_4 >= 0) then
               if(data_L4[11] < 0.3)then
					L4 = 0
				else
					L4 = 1
               end
		end
		if(L_5 >= 0) then
               if(data_L5[11] < 0.3)then
					L5 = 0
				else
					L5 = 1
               end
		end
		if(L_6 >= 0) then
               if(data_L6[11] < 0.3)then
					L6 = 0
				else
					L6 = 1
               end
		end
		if(L_7 >= 0) then
               if(data_L7[11] < 0.3)then
					L7 = 0
				else
					L7 = 1
               end
		end
		if(C_1 >= 0) then
               if(data_C1[11] < 0.3)then
					C1 = 0
				else
					C1 = 1
               end
		end
		if(C_2 >= 0) then
               if(data_C2[11] < 0.3)then
					C2 = 0
				else
					C2 = 1
               end
		end
		if(R_1 >= 0) then
               if(data_R1[11] < 0.3)then
					R1 = 0
				else
					R1 = 1
               end
		end
		if(R_2 >= 0) then
               if(data_R2[11] < 0.3)then
					R2 = 0
				else
					R2 = 1
               end
		end
		if(R_3 >= 0) then
               if(data_R3[11] < 0.3)then
					R3 = 0
				else
					R3 = 1
               end
		end
		if(R_4 >= 0) then
               if(data_R4[11] < 0.3)then
					R4 = 0
				else
					R4 = 1
               end
		end
		if(R_5 >= 0) then
               if(data_R5[11] < 0.3)then
					R5 = 0
				else
					R5 = 1
               end
		end
		if(R_6 >= 0) then
               if(data_R6[11] < 0.3)then
					R6 = 0
				else
					R6 = 1
               end
		end
		if(R_7 >= 0) then
               if(data_R7[11] < 0.3)then
					R7 = 0
				else
					R7 = 1
               end
		end

		-- Count the number of sensors on the line
		count_0 = 16 - (L1 + L2 + L3 + L4 + L5 + L6 + L7 + C1 + C2 + R1 + R2 + R3 + R4 + R5 + R6 + R7)
        if(count_0 ~= 0)then
            -- Calculate the degree of yaw
            error_1 = -(L1*(-14) + L2*(-12) + L3*(-10) + L4*(-8) + L5*(-6) + L6*(-4) + L7*(-2) + C1*0 + C2*0 + R1*2 + R2*4 + R3*6 + R4*8 + R5*10 + R6*12 + R7*14)/(count_0)
            error_0 = error_1/10
            error_Diff = error_0 - lastError
            -- Calculate the speed at which the motor should change
            motorspeed = (Kp*error_0) + (Kd*error_Diff)
            lastError = error_0
        else
            motorspeed = 0
		end


		-- Set the speed of the motor in the back mode
		if(drive == 2) then
            if(backUntilTime > sim.getSimulationTime()) then
                desiredLeftWheelRotSpeed = -1
                desiredRightWheelRotSpeed = -1
            else
				-- The car moves forward after the retreat is completed
                drive = 1
                desiredLeftWheelRotSpeed = 3
                desiredRightWheelRotSpeed = 3
            end
        -- Set the speed of the motor in the forward mode
        elseif(drive == 1) then
			desiredLeftWheelRotSpeed = desiredLeftBaseWheelRotSpeed + motorspeed
			desiredRightWheelRotSpeed = desiredRightBaseWheelRotSpeed - motorspeed
            if(desiredLeftWheelRotSpeed >= 3) then
                desiredLeftWheelRotSpeed = 3
            end
            if(desiredRightWheelRotSpeed >= 3) then
                desiredRightWheelRotSpeed = 3
            end
            if(desiredLeftWheelRotSpeed <= 0) then
                desiredLeftWheelRotSpeed = 0
            end
            if(desiredRightWheelRotSpeed <= 0) then
                desiredRightWheelRotSpeed = 0
            end
        end

        -- We take care of setting the desired wheel rotation speed:
        sim.setJointTargetVelocity(motorLeft,desiredLeftWheelRotSpeed)
        sim.setJointTargetVelocity(motorRight,desiredRightWheelRotSpeed)

        -- Since this script is threaded, don't waste time here:
        sim.switchThread() -- Resume the script at next simulation loop start
    end
end