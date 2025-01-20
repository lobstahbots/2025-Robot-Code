public class endEffector extends SubsystemBase{
    private final CANSparkMax endEffectorMotor;
    private final int MOTOR_SPEED;

    public endEffector(int endEffectorMotorID) {
        endEffectorMotor = new CANSparkMaxSparkMax(endEffectorMotorID);
        MOTOR_SPEED = 2
  }

  public void stopMotor() {
    endEffectorMotor.stopMotor();
    }

  public void runMotor () {
    endEffectorMotor.set(MOTOR_SPEED);
  }
}