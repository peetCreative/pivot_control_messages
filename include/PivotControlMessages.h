#ifndef PIVOT_CONTROL_MESSAGES_H
#define PIVOT_CONTROL_MESSAGES_H

#include <sstream>
#include <cmath>

namespace pivot_control_messages
{
    //! \brief Structure to define a pivoting movement by euler angles and entrence depth
    /*! The implementation may define the axis of the rotations and translations.
     * So one may define the Axes
     * by the axes of the laparoscope stick or by the direction of the tilted camera
     */
    struct DOFPose
    {
        //! rotation angle around x-Axis (Vertical movement in the image)
        double pitch = 0;
        //! rotation angle around y-Axis (Horizontal movement in the image)
        double yaw = 0;
        //! rotation angle around z-Axis (rotational movement in the image)
        double roll = 0;
        //! translation along the z-Axis (zooming the image)
        double transZ = 0;
        /*!
         *
         * @return human readable string
         */
        std::string toString() const
        {
            std::stringstream ss;
            ss << "pitch:" << pitch
               << " yaw:" << yaw
               << " roll:" << roll
               << " transZ:" << transZ;
            return ss.str();
        }
        //! \brief exact compare by value
        bool operator==(const DOFPose& other) const
        {
            return pitch == other.pitch &&
                   yaw == other.yaw &&
                   roll == other.roll &&
                   transZ == other.transZ;
        }
        //! \brief exact compare by value
        bool operator!=(const DOFPose& other) const
        {
            return !this->operator==(other);
        }
        //TODO: bring this to Eigen
        //! \brief compare using Epsilon tolerance
        /*!
         *
         * @param other other DOFPose object
         * @param rotEpsilon Epsilon to be used for the rotational values (pitch,yaw,roll)
         * @param transZEpsilon Epsilon to be used for the rotational values (pitch,yaw,roll)
         * @return is close to the other DOFPose object
         */
        bool closeTo(DOFPose &other, double rotEpsilon, double transZEpsilon) const
        {
            double diffPitch = pitch - other.pitch;
            double diffYaw = yaw - other.yaw;
            double diffRoll = roll - other.roll;
            double diffTransZ = transZ - other.transZ;
            double rotDist = std::sqrt(
                    diffPitch * diffPitch +
                    diffYaw * diffYaw +
                    diffRoll * diffRoll);
            double transZDist = std::abs(diffTransZ);
            return  rotDist < rotEpsilon && transZDist < transZEpsilon;
        }
    };
    //! \brief Structure Defining the Limits the Pivoting can move to at max/min
    struct DOFBoundaries
    {
        double pitchMax = 0;
        double pitchMin = 0;
        double yawMax = 0;
        double yawMin = 0;
        double rollMax = 0;
        double rollMin = 0;
        double transZMax = 0;
        double transZMin = 0;

        bool poseInside(DOFPose pose) const
        {
            return pose.pitch >= this->pitchMin &&
                pose.pitch <= this->pitchMax &&
                pose.yaw >= this->yawMin &&
                pose.yaw <= this->yawMax &&
                pose.roll >= this->rollMin &&
                pose.roll <= this->rollMax &&
                pose.transZ >= this->transZMin &&
                pose.transZ <= this->transZMax;
        }
    };

    class PivotController {
    protected:
        bool mDofPoseReady = false;
        bool mDofBoundariesReady = false;
    public:
        //! \brief sets the DOFPose the robot is supposed to move to
        virtual bool setTargetDOFPose(
                DOFPose) = 0;
        //! \brief gets the DOFPose the robot is currently (inflight) in
        virtual bool getCurrentDOFPose(
                DOFPose &laparoscopeDofPose) = 0;
        //! \brief gets the configured/determined Boundaries (DOFPose) the robot can move in
        virtual bool getDOFBoundaries(
                DOFBoundaries &laparoscopeDofBoundaries) = 0;
        //! \brief checks if the controller is ready to pivot
        bool isReady() const {
            return mDofBoundariesReady && mDofPoseReady;};
    };
}

#endif //PIVOT_CONTROL_MESSAGES_H