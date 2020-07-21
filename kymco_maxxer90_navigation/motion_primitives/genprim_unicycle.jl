#
# Copyright (c) 2008, Maxim Likhachev
# All rights reserved.
# 
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
# 
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the Carnegie Mellon University nor the names of its
#       contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES LOSS OF USE, DATA, OR PROFITS OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#

# Julia translation.

using Plots
using Printf
using LinearAlgebra
default(legend = false)

# Play with these based on your needs
# ------------ From here ------------
resolution = 0.05
numberofangles = 16 # preferably a power of 2, definitely multiple of 8
numberofprimsperangle = 5

# Multipliers (multiplier is used as costmult*cost)
forwardcostmult = 1
backwardcostmult = 50
forwardandturncostmult = 2
sidestepcostmult = 50
turninplacecostmult = 50

# ------------- To here -------------



function genmprim_unicycle(outfilename)
    # 0 degreees
    basemprimendpts0_c = zeros(numberofprimsperangle, 4) # x,y,theta,costmult 
    # x aligned with the heading of the robot, angles are positive
    # counterclockwise
    # 0 theta change
    basemprimendpts0_c[1,:] = [1,0,0,forwardcostmult]
    basemprimendpts0_c[2,:] = [8,0,0,forwardcostmult]
    basemprimendpts0_c[3,:] = [-1,0,0,backwardcostmult]    
    # 1/16 theta change
    basemprimendpts0_c[4,:] = [8,1,1,forwardandturncostmult]
    basemprimendpts0_c[5,:] = [8,-1,-1,forwardandturncostmult]
    # turn in place
    # basemprimendpts0_c[6,:] = [0,0,1,turninplacecostmult]
    # basemprimendpts0_c[7,:] = [0,0,-1,turninplacecostmult]
    
    # 45 degrees
    basemprimendpts45_c = zeros(numberofprimsperangle, 4) # x,y,theta,costmult (multiplier is used as costmult*cost)
    # x aligned with the heading of the robot, angles are positive
    # counterclockwise
    # 0 theta change 
    basemprimendpts45_c[1,:] = [1,1,0,forwardcostmult]
    basemprimendpts45_c[2,:] = [6,6,0,forwardcostmult]
    basemprimendpts45_c[3,:] = [-1,-1,0,backwardcostmult]    
    # 1/16 theta change
    basemprimendpts45_c[4,:] = [5,7,1,forwardandturncostmult]
    basemprimendpts45_c[5,:] = [7,5,-1,forwardandturncostmult]    
    # turn in place
    # basemprimendpts45_c[6,:] = [0,0,1,turninplacecostmult]
    # basemprimendpts45_c[7,:] = [0,0,-1,turninplacecostmult]
    
    # 22.5 degrees
    basemprimendpts22p5_c = zeros(numberofprimsperangle, 4) # x,y,theta,costmult (multiplier is used as costmult*cost)
    # x aligned with the heading of the robot, angles are positive
    # counterclockwise
    # 0 theta change     
    basemprimendpts22p5_c[1,:] = [2,1,0,forwardcostmult]
    basemprimendpts22p5_c[2,:] = [6,3,0,forwardcostmult]    
    basemprimendpts22p5_c[3,:] = [-2,-1,0,backwardcostmult]     
    # 1/16 theta change
    basemprimendpts22p5_c[4,:] = [5,4,1,forwardandturncostmult]
    basemprimendpts22p5_c[5,:] = [7,2,-1,forwardandturncostmult]    
    # turn in place
    # basemprimendpts22p5_c[6,:] = [0,0,1,turninplacecostmult]
    # basemprimendpts22p5_c[7,:] = [0,0,-1,turninplacecostmult]

    open(outfilename, "w") do f

        @printf(f, "resolution_m: %f\n", resolution)
        @printf(f, "numberofangles: %d\n", numberofangles)
        @printf(f, "totalnumberofprimitives: %d\n", numberofprimsperangle*numberofangles)

        for angleind in 1:numberofangles
            #text(----) TODO
            display(plot())

            for primind in 1:numberofprimsperangle
                @printf(f, "primID: %d\n", primind-1)
                @printf(f, "startangle_c: %d\n", angleind-1)

                # current angle
                currentangle = (angleind-1)*2*pi/numberofangles
                currentangle_36000int = round((angleind-1)*36000/numberofangles)
                
                # compute which template to use
                if (currentangle_36000int %  9000 == 0)
                    basemprimendpts_c = basemprimendpts0_c[primind,:]    
                    angle = currentangle
                elseif (currentangle_36000int % 4500 == 0)
                    basemprimendpts_c = basemprimendpts45_c[primind,:]
                    angle = currentangle - 45*pi/180
                elseif ((currentangle_36000int-7875) % 9000 == 0)
                    basemprimendpts_c = basemprimendpts33p75_c[primind,:]
                    basemprimendpts_c[1] = basemprimendpts33p75_c[primind, 2] # reverse x and y
                    basemprimendpts_c[2] = basemprimendpts33p75_c[primind, 1]
                    basemprimendpts_c[3] = -basemprimendpts33p75_c[primind, 3] # reverse the angle as well
                    angle = currentangle - 78.75*pi/180
                    println("78p75")
                elseif ((currentangle_36000int-6750) % 9000 == 0)
                    basemprimendpts_c = basemprimendpts22p5_c[primind,:]
                    basemprimendpts_c[1] = basemprimendpts22p5_c[primind, 2] # reverse x and y
                    basemprimendpts_c[2] = basemprimendpts22p5_c[primind, 1]
                    basemprimendpts_c[3] = -basemprimendpts22p5_c[primind, 3] # reverse the angle as well
                    angle = currentangle - 67.5*pi/180
                    println("67p5")            
                elseif ((currentangle_36000int-5625) % 9000 == 0)
                    basemprimendpts_c = basemprimendpts11p25_c[primind,:]
                    basemprimendpts_c[1] = basemprimendpts11p25_c[primind, 2] # reverse x and y
                    basemprimendpts_c[2] = basemprimendpts11p25_c[primind, 1]
                    basemprimendpts_c[3] = -basemprimendpts11p25_c[primind, 3] # reverse the angle as well
                    angle = currentangle - 56.25*pi/180
                    println("56p25")
                elseif ((currentangle_36000int-3375) % 9000 == 0)
                    basemprimendpts_c = basemprimendpts33p75_c[primind,:]
                    angle = currentangle - 33.75*pi/180
                    println("33p75")
                elseif ((currentangle_36000int-2250) % 9000 == 0)
                    basemprimendpts_c = basemprimendpts22p5_c[primind,:]
                    angle = currentangle - 22.5*pi/180
                    println("22p5")
                elseif ((currentangle_36000int-1125) % 9000 == 0)
                    basemprimendpts_c = basemprimendpts11p25_c[primind,:]
                    angle = currentangle - 11.25*pi/180
                    println("11p25")
                else
                    println("ERROR: invalid angular resolution. angle = $(currentangle_36000int)")
                    return
                end

                # now figure out what action will be        
                baseendpose_c = basemprimendpts_c[1:3]
                additionalactioncostmult = basemprimendpts_c[4]
                endx_c = round(baseendpose_c[1]*cos(angle) - baseendpose_c[2]*sin(angle))        
                endy_c = round(baseendpose_c[1]*sin(angle) + baseendpose_c[2]*cos(angle))
                endtheta_c = (angleind - 1 + baseendpose_c[3]) % numberofangles
                endpose_c = [endx_c endy_c endtheta_c]
                
                println("rotation angle=$(angle*180/pi)")
                
                if baseendpose_c[2] == 0 && baseendpose_c[3] == 0
                    # println("endpose=$(endpose_c[1]) $(endpose_c[2]) $(endpose_c[3])")
                end
                
                # generate intermediate poses (remember they are w.r.t 0,0 (and not
                # centers of the cells)
                numofsamples = 10
                intermcells_m = zeros(numofsamples,3)
                startpt = [0,0,currentangle]
                endpt = [endpose_c[1]*resolution,endpose_c[2]*resolution,((angleind - 1 + baseendpose_c[3]) % numberofangles)*2*pi/numberofangles]
                intermcells_m = zeros(numofsamples,3)
                if ((endx_c == 0 && endy_c == 0) || baseendpose_c[3] == 0) # turn in place or move forward            
                    for iind in 1:numofsamples
                        intermcells_m[iind,:] = [startpt[1] + (endpt[1] - startpt[1])*(iind-1)/(numofsamples-1),startpt[2] + (endpt[2] - startpt[2])*(iind-1)/(numofsamples-1),0]
                        rotation_angle = baseendpose_c[3] * (2*pi/numberofangles)
                        intermcells_m[iind,3] = (startpt[3] + (rotation_angle)*(iind-1)/(numofsamples-1)) % (2*pi)
                    end            
                else # unicycle-based move forward or backward
                    R = [cos(startpt[3]) sin(endpt[3])-sin(startpt[3]);sin(startpt[3]) -(cos(endpt[3]) -cos(startpt[3]))]
                    S = pinv(R)*[endpt[1] - startpt[1],endpt[2] - startpt[2]]
                    l = S[1] 
                    tvoverrv = S[2]
                    rv = (baseendpose_c[3]*2*pi/numberofangles + l/tvoverrv)
                    tv = tvoverrv*rv

                    if l < 0
                        println("WARNING: l = $l < 0 -> bad action start/end points")
                        l = 0
                    end
                    # compute rv
                    # rv = baseendpose_c[3]*2*pi/numberofangles
                    # compute tv
                    # tvx = (endpt[1] - startpt[1])*rv/(sin(endpt[3]) - sin(startpt[3]))
                    # tvy = -(endpt[2] - startpt[2])*rv/(cos(endpt[3]) - cos(startpt[3]))
                    # tv = (tvx + tvy)/2.0              
                    # generate samples
                    for iind = 1:numofsamples                                        
                        dt = (iind-1)/(numofsamples-1)

                        if dt*tv < l
                            intermcells_m[iind,:] = [startpt[1] + dt*tv*cos(startpt[3]),startpt[2] + dt*tv*sin(startpt[3]),startpt[3]]
                        else
                            dtheta = rv*(dt - l/tv) + startpt[3]
                            intermcells_m[iind,:] = [startpt[1] + l*cos(startpt[3]) + tvoverrv*(sin(dtheta) - sin(startpt[3])),startpt[2] + l*sin(startpt[3]) - tvoverrv*(cos(dtheta) - cos(startpt[3])),dtheta]
                        end
                    end 
                    # correct
                    errorxy = [endpt[1] - intermcells_m[numofsamples,1] 
                               endpt[2] - intermcells_m[numofsamples,2]]
                    println("l=$l errx=$(errorxy[1]) erry=$(errorxy[2])")
                    interpfactor = collect(0:1/(numofsamples-1):1)
                    intermcells_m[:,1] = intermcells_m[:,1] + errorxy[1]*interpfactor
                    intermcells_m[:,2] = intermcells_m[:,2] + errorxy[2]*interpfactor
                end                                        
                # write out
                @printf(f, "endpose_c: %d %d %d\n", endpose_c[1], endpose_c[2], endpose_c[3])
                @printf(f, "additionalactioncostmult: %d\n", additionalactioncostmult)
                @printf(f, "intermediateposes: %d\n", size(intermcells_m,1))
                for interind in 1:size(intermcells_m, 1)
                    @printf(f, "%.4f %.4f %.4f\n", intermcells_m[interind,1], intermcells_m[interind,2], intermcells_m[interind,3]);
                end
                display(plot!(intermcells_m[:,1], intermcells_m[:,2], xlim=(-0.3,0.3), ylim=(-0.3,0.3)))
            end
            readline()
        end

    end
end

if length(ARGS) < 1
    println("Ooooops!")
    exit()
end

genmprim_unicycle(ARGS[1])


