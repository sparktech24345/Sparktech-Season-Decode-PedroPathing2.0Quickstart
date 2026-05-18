#include <jni.h>
#include <cmath>

// Write C++ code here.
//
// Do not forget to dynamically load the C++ library into your application.
//
// For instance,
//
// In MainActivity.java:
//    static {
//       System.loadLibrary("ftcrobotcontroller");
//    }
//
// Or, in MainActivity.kt:
//    companion object {
//      init {
//         System.loadLibrary("ftcrobotcontroller")
//      }
//    }

extern "C" {
    double mmax(double a1, double a2) {
        return a1 > a2 ? a1 : a2;
    }
    double mmin(double a1, double a2) {
        return a1 < a2 ? a1 : a2;
    }
    double mabs(double a) {
        return a < 0 ? -a : a;
    }
    double mclamp(double val, double mini, double maxi) {
        return mmax(mmin(val, maxi), mini);
    }
}

extern "C"
JNIEXPORT jdouble JNICALL
Java_org_firstinspires_ftc_teamcode_Experimental_HelperClasses_Math_max(JNIEnv *env, jclass clazz,
                                                                        jdouble a1, jdouble a2) {
    return mmax(a1, a2);
}
extern "C"
JNIEXPORT jdouble JNICALL
Java_org_firstinspires_ftc_teamcode_Experimental_HelperClasses_Math_min(JNIEnv *env, jclass clazz,
                                                                        jdouble a1, jdouble a2) {
    return mmin(a1, a2);
}
extern "C"
JNIEXPORT jdouble JNICALL
Java_org_firstinspires_ftc_teamcode_Experimental_HelperClasses_Math_abs(JNIEnv *env, jclass clazz,
                                                                        jdouble a) {
    return mabs(a);
}
extern "C"
JNIEXPORT jboolean JNICALL
Java_org_firstinspires_ftc_teamcode_Experimental_HelperClasses_Math_eval__D(JNIEnv *env,
                                                                            jclass clazz,
                                                                            jdouble a) {
    return a != 0;
}
extern "C"
JNIEXPORT jdouble JNICALL
Java_org_firstinspires_ftc_teamcode_Experimental_HelperClasses_Math_eval__Z(JNIEnv *env,
                                                                            jclass clazz,
                                                                            jboolean a) {
    return a;
}
extern "C"
JNIEXPORT jboolean JNICALL
Java_org_firstinspires_ftc_teamcode_Experimental_HelperClasses_Math_triggerEval(JNIEnv *env,
                                                                                jclass clazz,
                                                                                jdouble t) {
    return t >= 0.4;
}
extern "C"
JNIEXPORT jdouble JNICALL
Java_org_firstinspires_ftc_teamcode_Experimental_HelperClasses_Math_sqrt(JNIEnv *env, jclass clazz,
                                                                         jdouble a) {
    return std::sqrt(a);
}

extern "C"
double callDoubleGetter(JNIEnv *env, jobject obj, const char *name) {
    jclass cls = env->GetObjectClass(obj);
    jmethodID jmethodId = env->GetMethodID(cls, name, "()D");
    return env->CallDoubleMethod(obj, jmethodId);
}

extern "C"
JNIEXPORT jdouble JNICALL
Java_org_firstinspires_ftc_teamcode_Experimental_HelperClasses_Math_calculateDistance(JNIEnv *env,
                                                                                      jclass clazz,
                                                                                      jobject pose1,
                                                                                      jobject pose2,
                                                                                      jboolean convert_to_meters) {
        double correctedX = callDoubleGetter(env, pose1, "getX");
        if (correctedX < 0) correctedX = 0; /// if 0 is less than 0 that means you have exited field, this might be risky though

        double dx = callDoubleGetter(env, pose2, "getX") - correctedX;
        double dy = callDoubleGetter(env, pose2, "getY") - callDoubleGetter(env, pose1, "getY");

        double distance = std::sqrt(dx * dx + dy * dy);

        if (convert_to_meters)
            // Assuming your Pose coordinates are in INCHES (FTC standard)
            distance *= 0.0254; // inches to meters

        return distance;
}
extern "C"
JNIEXPORT jdouble JNICALL
Java_org_firstinspires_ftc_teamcode_Experimental_HelperClasses_Math_calculateDistanceNonZero(
        JNIEnv *env, jclass clazz, jobject pose1, jobject pose2, jboolean convert_to_meters) {
    double distance = Java_org_firstinspires_ftc_teamcode_Experimental_HelperClasses_Math_calculateDistance(env, clazz, pose1, pose2, convert_to_meters);
    if (distance == 0) return 0.00001;
    return distance;
}

constexpr double realDown = 76; // unghi real, masurate in cad, a nu se schimba
constexpr double realUp = 58; // tot areal, unghi pana la blocker mecanic

constexpr double servoDown = 136.8; // servo down * 360 actually 0.38
constexpr double servoUp = 36; // servo up * 360 actually 0.1

constexpr double realDiff = realUp - realDown;
constexpr double servoDiff = servoUp - servoDown;
constexpr double inverseRealDiff = 1.0 / realDiff;


extern "C"
JNIEXPORT jdouble JNICALL
Java_org_firstinspires_ftc_teamcode_Experimental_HelperClasses_Math_degreesToOuttakeTurretServo(
        JNIEnv *env, jclass clazz, jdouble degrees) {
    degrees = mclamp(degrees, realUp, realDown);

    //interpolation
    return servoDown + (degrees - realDown) * servoDiff * inverseRealDiff;
}

constexpr double voltajA = 12.7;
constexpr double multiplierA = 0.9;

constexpr double voltajB = 12;
constexpr double multiplierB = 1;

constexpr double voltajDiff = voltajB - voltajA;
constexpr double inverseVoltajDiff = 1.0 / voltajDiff;
constexpr double multiplierDiff = multiplierB - multiplierA;

extern "C"
JNIEXPORT jdouble JNICALL
Java_org_firstinspires_ftc_teamcode_Experimental_HelperClasses_Math_voltageMultiplierForMotor(
        JNIEnv *env, jclass clazz, jdouble voltage) {
    voltage = mclamp(voltage,11,14);

    return multiplierA + (voltage - voltajA) * multiplierDiff * inverseVoltajDiff;
}