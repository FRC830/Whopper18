#include "Shooter.h"

Shooter::Shooter(WPI_TalonSRX &intake, WPI_TalonSRX &winch, Servo &servo):
    m_intake(intake), m_winch(winch), m_servo(servo) {}

// Sets the speed of the Winch (can be negative)
void Shooter::Angle(float change){
    if (!limit.Get()){
        m_winch.Set(change);
    }
    else{
        m_winch.Set(0);
    }
}
// Turns on/off the m1 intakeWheels to spin outward based on a bool
void Shooter::Shooting(bool outtaking){
    if (outtaking) {
        timer.Start();
        m_intake.Set(1);
        if (timer.Get() >= 0.5){
            m_servo.Set(1);
        }
    } else {
        timer.Stop();
        timer.Reset();
        m_intake.Set(0);
        m_servo.Set(0);
    }
}
// Turns on/off the m1 intakeWheels to spin inward based on a bool
void Shooter::Intake(bool intaking){
    if (intaking){
        m_intake.Set(-1);
    } else {
        m_intake.Set(0);
    }
}