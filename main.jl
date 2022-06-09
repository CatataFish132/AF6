using LibSerialPort
using FFTW
using Plots
using Rotations
using LinearAlgebra
using SmoothLivePlot

# initialize all the variables
x = Array{Float64,1}(undef, 0)
y = Array{Float64,1}(undef, 0)
z = Array{Float64,1}(undef, 0)
roll = Array{Float64,1}(undef, 0)
pitch = Array{Float64,1}(undef, 0)
yaw = Array{Float64,1}(undef, 0)

output_roll = Array{Float64,1}(undef, 0)
output_pitch = Array{Float64,1}(undef, 0)
output_yaw = Array{Float64,1}(undef, 0)
output_acc_roll = Array{Float64,1}(undef, 0)
output_acc_pitch = Array{Float64,1}(undef, 0)

# goes from a euler to a quaternion
function euler_to_quaternion(roll, pitch, yaw)
    a = RotXYZ(roll, pitch, yaw)
    b = UnitQuaternion(a)
    [b.w; b.x; b.y; b.z]
end

# goes from a quaternion to a euler
function quaternion_to_euler(q)
    b = UnitQuaternion(q[1], q[2], q[3], q[4])
    c = RotXYZ(b)
    [c.theta1
        c.theta2
        c.theta3]
end

# the live plot function
function plotting(roll, pitch, yaw, output_acc_roll, output_acc_pitch)
    sleep(0.001)
    p1 = plot(roll, ylim=(-1, 1), label="roll")
    plot!(pitch, label="pitch")
    plot!(yaw, label="yaw")
    p1
end

# make the live plot object
outPlotObject = @makeLivePlot plotting([0], [0], [0], [0], [0])
modifyPlotObject!(outPlotObject, arg1 = [0], arg2 = [0], arg3 = [0], arg4 = [0], arg5 = [0])

# creates the filter coefficients for a low pass filter
function create_low_pass(Wco, n)
    h = []
    v = Float64
    for i in -n:n
        if i == 0
            v = Wco / pi
        else
            v = sin(Wco * i) / (i * pi)
        end
        push!(h, v)
    end
    return h
end

# filters the signal with the given filter
function dsp(signal, filter)
    output = 0
    for i in 1:length(filter)
        output += signal[i]*filter[i]
    end
    return output
end

# setting inital matrices
x_hat = [[1.0; 0.0; 0.0; 0.0], []]
# creates identity matrix
H = Matrix(1I, 4, 4)
# creates a 4x4 identity matrix
P = [Matrix(1.0I, 4, 4), []]
# creates a 4x4 matrix with scalar 10^-4
Q = Matrix(10e-4I, 4, 4)
# creates a 4x4 matrix with scalar 1
R = Matrix(1I, 4, 4)

HT = transpose(H)
g = 9.81
dt = 0.01
i = 0

# cutoff frequency at 10 hz
filter = create_low_pass(pi / 5, 20)

# function where all the filtering is done
function filtering(x,y,z, roll, pitch, yaw)
    global i
    # filters the accelerometer measurements
    x_d = dsp(x, filter)
    y_d = dsp(y, filter)
    # z_d = dsp(z, filter)

    # accelerations to roll and pitch

    # the if statemens make sure it doesn't crash when you move the IMU up or down because we cant take the asin if |x|>1
    if abs(x_d/9.81) > 1
        acc_pitch = -asin(sign(x_d))
    else
        acc_pitch = -asin(x_d / g)
    end

    if abs(-y_d / g * cos(acc_pitch)) > 1
        acc_roll = -asin(sign(-y_d / g * cos(acc_pitch)))
    else
        acc_roll = -asin(-y_d / g * cos(acc_pitch))
    end

    x = [0 roll pitch yaw]

    # insert gyro data
    A = I + (dt / 2) * [x[1] -x[2] -x[3] -x[4]
        x[2] x[1] x[4] -x[3]
        x[3] -x[4] x[1] x[2]
        x[4] x[3] -x[2] x[1]]

    # predict the state
    x_hat[2] = A * x_hat[1]

    # skip kalman filter for yaw because we cant measure it via accelerometer
    push!(output_yaw, quaternion_to_euler(x_hat[2])[3])

    # predict the error
    P[2] = A * P[1] * transpose(A) + Q

    # Compute the Kalman gain
    K = P[2] * HT * inv(H * P[2] * HT + R)

    Z = euler_to_quaternion(acc_roll, acc_pitch, 0)

    # Compute the new estimate
    x_hat[2] = x_hat[2] + K * (Z - H * x_hat[2])

    # compute the new error Covariance
    P[2] = P[2] - K * H * P[2]

    # set P[1] to P[2] for the next loop
    P[1] = P[2]

    output = quaternion_to_euler(x_hat[2])
    # append each value to the corresponding list so we can plot it
    push!(output_roll, output[1])
    push!(output_pitch, output[2])
    push!(output_acc_roll, acc_roll)
    push!(output_acc_pitch, acc_pitch)

    # skip yaw correction
    x_hat[1] = euler_to_quaternion(output[1], output[2], output_yaw[end])

    # delete the oldest value if we have more than 200 values
    if length(output_roll) > 200
        popfirst!(output_roll)
        popfirst!(output_pitch)
        popfirst!(output_yaw)
        popfirst!(output_acc_roll)
        popfirst!(output_acc_pitch)
    end

    # plot every 2th loop
    if i == 2
        modifyPlotObject!(outPlotObject, arg1 = output_roll, arg2 = output_pitch, arg3 = output_yaw, arg4 = output_acc_roll, arg5 = output_acc_pitch)
        i = -1
    end
    i+=1
end

# connect to Serial
portname = "/dev/ttyUSB1"
baudrate = 115200
sp = LibSerialPort.open(portname, baudrate)
sleep(0.1)
# reset values
values = ""
while bytesavailable(sp) < 100
    continue
end
values = String(read(sp))
values = values[findlast(isequal(';'), values)+1:end]

# getting values from the serial and calling the filtering function with those values
while true
    global values
    if bytesavailable(sp) > 0
        values = values * String(read(sp))
        if in(';', values)
            i = findfirst(isequal(';'), values)
            finalized_values = values[1:i-1]
            values = values[i+1:end]
            num = split(finalized_values, ',')
            number_list = Array{Float64,1}(undef, 0)
            if length(num) < 6 || num[1] == ""
                continue
            end
            for number_str in num
                push!(number_list, parse(Float64, number_str))
            end
            push!(x, number_list[1])
            push!(y, number_list[2])
            push!(z, number_list[3])
            push!(roll, number_list[4])
            push!(pitch, number_list[5])
            push!(yaw, number_list[6])
            if length(x) > 42
                # calling the filtering function which uses DSP filtering and kalman filtering and then plots the results
                filtering(x,y,z, roll[20], pitch[20], yaw[20])
                popfirst!(x)
                popfirst!(y)
                popfirst!(z)
                popfirst!(roll)
                popfirst!(pitch)
                popfirst!(yaw)
            end
        end
    end
end