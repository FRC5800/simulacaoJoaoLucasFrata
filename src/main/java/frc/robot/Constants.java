package frc.robot;

public final class Constants {
    public static class ConstantsDriveTrain {
        public static final int kLeftMaster = 1;
        public static final int kRightMaster = 2;
        public static final int kLeftSlave = 3;
        public static final int kRightSlave = 4;
    }
    public static class ConstantsElevador{
        public static int Master_ID_Elevador = 6;
        public static int Slave_ID_Elevador = 7;
    
        public static double voltage_Elevator = 0;
        public static double kp = 0.05;
        public static double ki = 0.0;
        public static double kd = 0.0;
        public static double pidTolerencia = 0.5;
        public static double calculo_Ticks_Elevador = 0.7950125000000001 * 5.14;
      }
}
