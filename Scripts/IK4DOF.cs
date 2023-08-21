using System.Collections;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;

public class IK4DOF : MonoBehaviour
{
    public float L1 = 136; //millimeters
    public float L2 = 99; //millimeters

    public float simulationScale = 1/10;

    public float J1Angle;
    public float J2Angle;
    public float J3Angle;
    public float J4Angle;

    //For simulation. J1 is the base (y-axis), J2, J3, and J4 are the scissor lift joints (x-axis). End effector on J4:
    public Transform J1Object;
    public Transform J2Object;
    public Transform J3Object;
    public Transform J4Object;

    //Where the end effector should go:
    public Transform reference;


    // Update is called once per frame
    void Update()
    {
        float xDistance = new Vector3(reference.position.x - J1Object.position.x, 0, reference.position.z - J1Object.position.z).magnitude;
        float zDistance = reference.position.y - J1Object.position.y;

        Vector3 referencePosition = new Vector3(0, zDistance, xDistance);

        CalculateJoint1Angle(J1Object.position, reference.position);

        float[] angles = CalculateJoint24Angles(referencePosition);
        //Debug.Log("J1: " + GetJoint1Angle() + ", J2: " + (angles[0] - 180) + ", J3: " + angles[1] + ", J4: " + angles[2]);

        J1Angle = GetJoint1Angle();
        J2Angle = angles[0];
        J3Angle = angles[1] * -1;
        J4Angle = angles[2];


        J2Object.localRotation = Quaternion.Euler(new Vector3(angles[0], 0, 0));
        J3Object.localRotation = Quaternion.Euler(new Vector3(angles[1], 0, 0));
        J4Object.localRotation = Quaternion.Euler(new Vector3(angles[2], 0, 0));
    }


    float CalculateJoint1Angle(Vector3 origin, Vector3 targetPosition)
    {
        Vector3 lookPos = targetPosition - origin;
        lookPos.y = 0;
        J1Object.rotation =  Quaternion.LookRotation(lookPos) * Quaternion.AngleAxis(180, Vector3.up);

        return Quaternion.LookRotation(lookPos).eulerAngles.y;
    }

    public float GetJoint1Angle()
    {
        float angle = CalculateJoint1Angle(J1Object.position, reference.position);

        if (0 <= angle && angle <= 180)
        {
            angle = angle;
        }
        else
        {
            angle = angle - 360;
        }
        return angle;
    }


    /* Angles:
     *  a, b, c, alphaH
     * 
     * Physical bits:
     *  L1, L2
     * 
     * Joint Axii:
     *  J1: Y
     *  J2: X
     *  J3: X
     *  J4: X
     * 
     * 
     * Triangle L1L2H:
     * 
     *           b
     *         :    : L2
     *   L1  :         :
     *     :          ....c
     *   :     ...H''
     * a....''    
     * 
     * 
     * Triangle HxHyH:
     * 
     *                ...::
     *           H..:     : Hy
     *      ...:        ..:
     * ...:.............:.:
     * ^alphaH    Hx
     * 
     * 
     * 
     * Robot arm diagram:
     * 
     * 
     *                J3
     *       L1   :::   ::
     *         :::        :: L2
     *      :::             ::        ,--
     *   :::                  J4 :::::     End effector
     * J2                             '--
     * J1
     * Base
     * 
     * 
     */



    float[] CalculateJoint24Angles(Vector3 position)
    {
        //Coordinates to go to (simulationScale is if your Unity model is shrunk [like 8.0 tall instead of 80 if you have an 80mm arm])
        float Hx = position.z * simulationScale;
        float Hy = position.y * simulationScale;

        //Hypotenuse of triangle HxHyH:
        float H = Mathf.Sqrt(Hx * Hx + Hy * Hy);

        //Funny little angle that is one of the angles in triangle HxHyH:
        float alphaH = Mathf.Atan2(Hy, Hx) * Mathf.Rad2Deg;

        //Triangle L1L2H:
        float a = Mathf.Acos(Mathf.Clamp((L2 * L2 - L1 * L1 - H * H) / (-2 * L1 * H), -1, 1)) * Mathf.Rad2Deg;
        float b = Mathf.Acos(Mathf.Clamp((H * H - L1 * L1 - L2 * L2) / (-2 * L1 * L2), -1, 1)) * Mathf.Rad2Deg;
        float c = 180 - a - b;

        //The actual angles:
        float J2 = alphaH + a - 90;
        float J3 = b - 180;
        float J4 = 180 - alphaH + c + 180;

        float[] angles = new float[] { J2, J3, J4 };

        return angles;
    }
}