#include <math.h>
#include <iostream>
#include <string.h>

using namespace std;

class Vector2D
{
public:
    double x;
    double y;
};

// Prototypes
bool VectorInUpperQuadrants(Vector2D vector);
Vector2D vectorByAngle(double angle);
Vector2D rotateVectorByAngle(double angle, Vector2D vector);

bool IsClockwise(double angleActual, double angleDesired)
{
    Vector2D vectorActual = vectorByAngle(angleActual);
    Vector2D vectorDesired = vectorByAngle(angleDesired);
    /*
    string msg1 = "A: " + to_string(vectorActual.x) + ", " + to_string(vectorActual.y);
    string msg2 = "D: " + to_string(vectorDesired.x) + ", " + to_string(vectorDesired.y);
    cout << "Actual: " << msg1 << "\n";
    cout << "Desired: " << msg2 << "\n";
    */

    Vector2D rotatedVectorActual = rotateVectorByAngle(angleActual, vectorActual);
    Vector2D rotatedVectorDesired = rotateVectorByAngle(angleActual, vectorDesired);
    /*
    string msg3 = "A: " + to_string(rotatedVectorActual.x) + ", " + to_string(rotatedVectorActual.y);
    string msg4 = "D: " + to_string(rotatedVectorDesired.x) + ", " + to_string(rotatedVectorDesired.y);
    cout << "ROTATED:"
         << "\n";
    cout << "Actual: " << msg3 << "\n";
    cout << "Desired: " << msg4 << "\n\n\n";
    */

    return !VectorInUpperQuadrants(rotatedVectorDesired);
}

bool VectorInUpperQuadrants(Vector2D vector)
{
    return vector.y >= 0;
}

Vector2D rotateVectorByAngle(double angle, Vector2D vector)
{
    Vector2D rotatedVector;
    rotatedVector.x = vector.x * cos(2 * M_PI - angle) + vector.y * (-sin(2.0 * M_PI - angle));
    rotatedVector.y = vector.x * sin(2 * M_PI - angle) + vector.y * cos(2.0 * M_PI - angle);
    return rotatedVector;
}

Vector2D vectorByAngle(double angle)
{
    Vector2D vector;
    vector.x = cos(angle);
    vector.y = sin(angle);
    return vector;
}

/*bool IsAsExpected(bool b1, bool b2)
{
    return b1 == b2;
}*/

int main(int argc, char *argv[])
{
    /*cout << cos((27.0 / 23.0) * M_PI) << "\n\n\n";
    cout << IsClockwise(M_PI_2, (27.0 / 23.0) * M_PI);
    return 1;*/

    bool answers[20];
    answers[0] = IsClockwise(M_PI_2, (27.0 / 23.0) * M_PI);            //false
    answers[1] = IsClockwise((27.0 / 23.0) * M_PI, M_PI_2);            //true
    answers[2] = IsClockwise((17.0 / 9.0) * M_PI, (1.0 / 3.0) * M_PI); //false
    answers[3] = IsClockwise((1.0 / 3.0) * M_PI, (17.0 / 9.0) * M_PI); //true
    answers[4] = IsClockwise(M_PI_4, (6.0 / 5.0) * M_PI);              //false
    answers[5] = IsClockwise((6.0 / 5.0) * M_PI, M_PI_4);              //true
    answers[6] = IsClockwise(1.3 * M_PI, 0.2 * M_PI);                  //false
    answers[7] = IsClockwise(0.2 * M_PI, 1.3 * M_PI);                  //true
    answers[8] = IsClockwise(2.4, 1.2);                                //true
    answers[9] = IsClockwise(5.6, 1.2);                                //false
    /*
    answers[10] = IsAsExpected(IsClockwise(1.2, 2.4), false);                               //false
    answers[11] = IsAsExpected(IsClockwise(1.2, 5.6), true);                                //true
    answers[12] = IsAsExpected(IsClockwise(M_PI, 1.01 * M_PI), false);                      //false
    answers[13] = IsAsExpected(IsClockwise(1.01 * M_PI_2, M_PI_2), true);                   //true
    answers[14] = IsAsExpected(IsClockwise(M_PI_2, 3 * M_PI_2), false);                     //false
    answers[15] = IsAsExpected(IsClockwise(3 * M_PI_2, M_PI_2), false);                     //false
    answers[16] = IsAsExpected(IsClockwise(3 * M_PI, M_PI_2), true);                        //true
    answers[17] = IsAsExpected(IsClockwise(M_PI_2, 3 * M_PI), false);                       //false
    answers[18] = IsAsExpected(IsClockwise(5.5 * M_PI_2, 5.9 * M_PI), true);                //true
    answers[19] = IsAsExpected(IsClockwise(5.5 * M_PI_2, 5.4 * M_PI), false);               //false
    */

    int arrSize = sizeof(answers) / sizeof(answers[0]);

    for (int i = 0; i < 10; i++)
    {
        /*
        if (!answers[i])
            cout << "Mistake in: " << i << "\n";
        */

        string msg = answers[i] ? "true" : "false";
        cout << i << ": " << msg << "\n";
    }

    return 0;
}