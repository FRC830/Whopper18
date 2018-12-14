#ifndef SRC_SHOOTER_H_
#define SRC_SHOOTER_H_

#include <WPILib.h>
#include <ctre/Phoenix.h>
#include <DigitalInput.h>


class Shooter {
    public:
        void Intake(bool intaking);
        void Shoot(bool shooting);
        void Flywheel(bool shooting);
        void Angle(float change);
        Shooter(WPI_TalonSRX &intake, WPI_TalonSRX &winch, Servo &servo);


    private:
        static const int SWITCH_DIO = 0; //CHANGE
        WPI_TalonSRX &m_intake;
        WPI_TalonSRX &m_winch;
        Servo &m_servo;
        Timer timer;
        DigitalInput limit{SWITCH_DIO};
};

#endif