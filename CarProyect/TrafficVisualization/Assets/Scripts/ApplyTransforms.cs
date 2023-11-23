//Lucía Barrenechea
//13 de noviembre del 2023
//Descripción: Este script aplica las transformaciones a un objeto. Simula el movimiento de un auto y sus llantas.
using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class ApplyTransforms : MonoBehaviour
{
    [SerializeField] Vector3 displacement;
    [SerializeField]float scale;
    //[SerializeField]float angle;
    [SerializeField]AXIS rotationAxis;
    [SerializeField]AXIS wheelAxis;
    [SerializeField] GameObject wheelobj1;
    [SerializeField] GameObject wheelobj2;
    [SerializeField] GameObject wheelobj3;
    [SerializeField] GameObject wheelobj4;
    Mesh mesh;
    Mesh wheel1;
    Mesh wheel2;
    Mesh wheel3;
    Mesh wheel4;
    Vector3[] baseVertices;
    Vector3[] baseVerticesw1;
    Vector3[] baseVerticesw2;
    Vector3[] baseVerticesw3;
    Vector3[] baseVerticesw4;
    Vector3[] newVertices;
    Vector3[] newVerticesw1;
    Vector3[] newVerticesw2;
    Vector3[] newVerticesw3;
    Vector3[] newVerticesw4;
    // Start is called before the first frame update
    void Start()
    {
        mesh = GetComponentInChildren<MeshFilter>().mesh;
        wheel1 = wheelobj1.GetComponentInChildren<MeshFilter>().mesh;
        wheel2 = wheelobj2.GetComponentInChildren<MeshFilter>().mesh;
        wheel3 = wheelobj3.GetComponentInChildren<MeshFilter>().mesh;
        wheel4 = wheelobj4.GetComponentInChildren<MeshFilter>().mesh;
        baseVertices = mesh.vertices;
        baseVerticesw1 = wheel1.vertices;
        baseVerticesw2 = wheel2.vertices;
        baseVerticesw3 = wheel3.vertices;
        baseVerticesw4 = wheel4.vertices;

        newVertices = new Vector3[baseVertices.Length];
        for (int i = 0; i < baseVertices.Length; i++)
        {
            newVertices[i] = baseVertices[i];
        }
        
        newVerticesw1 = new Vector3[baseVerticesw1.Length];
        for (int i = 0; i < baseVerticesw1.Length; i++)
        {
            newVerticesw1[i] = baseVerticesw1[i];
        } 
        newVerticesw2 = new Vector3[baseVerticesw2.Length];
        for (int i = 0; i < baseVerticesw2.Length; i++)
        {
            newVerticesw2[i] = baseVerticesw2[i];
        } 
        newVerticesw3 = new Vector3[baseVerticesw3.Length];
        for (int i = 0; i < baseVerticesw3.Length; i++)
        {
            newVerticesw3[i] = baseVerticesw3[i];
        } 

        newVerticesw4 = new Vector3[baseVerticesw4.Length];
         for (int i = 0; i < baseVerticesw4.Length; i++)
        {
            newVerticesw4[i] = baseVerticesw4[i];
        } 
    }

    // Update is called once per frame
    void Update()
    {
        DoTransform();
    }

    void DoTransform(){
        T=getT();
        Vector3 newposition=PositionLerp(startPosition, endPosition, T);
        //Calulate the angle of rotation
        float anglerad = Mathf.Atan2(displacement.z, displacement.x);
        float angle = anglerad * Mathf.Rad2Deg -90;
        Debug.Log(angle);
        //create the matrices
        Matrix4x4 move= HW_Transforms.TranslationMat(newposition.x  , newposition.y , newposition.z);
        //HAcer otra función que tome posición iniical y final y que tiempo hay entre 0 y 1
        //Matrix4x4 move= HW_Transforms.TranslationMat(-displacement.x  , -displacement.y , -displacement.z );
        Matrix4x4 moveOrigin = HW_Transforms.TranslationMat(-displacement.x, -displacement.y, -displacement.z);
        Matrix4x4 moveObject = HW_Transforms.TranslationMat(displacement.x, displacement.y, displacement.z);
        Matrix4x4 rotate = HW_Transforms.RotateMat(angle , rotationAxis);
        
        //Permite que las ruedas roten sobre su propio eje x.
        Matrix4x4 wheelrotate = HW_Transforms.RotateMat(angle * Time.time, wheelAxis);
        //operations are executed in backwards order
        Matrix4x4 composite =  move * rotate;
        for (int i=0; i<newVertices.Length; i++)
        {
            Vector4 temp = new Vector4(baseVertices[i].x, baseVertices[i].y, baseVertices[i].z, 1);
            newVertices[i] = composite * temp;
        }
        
        //Crear 4 matrices de traslación respecto al coche. x-x, y-y, z-z.
        //adelante izquierda
        Matrix4x4 posicionI = HW_Transforms.TranslationMat(-1f, .3f, +1.5f);
        //adelante derecha
        Matrix4x4 posicionD = HW_Transforms.TranslationMat(+1f, .3f,+1.5f);
        //atras izquierda
        Matrix4x4 posicionIat = HW_Transforms.TranslationMat(-1f, .3f, -1.5f);
        //atras derecha
        Matrix4x4 posicionDat = HW_Transforms.TranslationMat(+1f, .3f, -1.5f);
        //Crear matriz de escala para las ruedas.
        Matrix4x4 scale2 = HW_Transforms.ScaleMat(scale, scale, scale);

        //Crea las matrices de los vertices de las ruedas.
        for (int i=0; i<newVerticesw1.Length; i++)
        {
            Vector4 tempw1 = new Vector4(baseVerticesw1[i].x, baseVerticesw1[i].y, baseVerticesw1[i].z, 1);
            newVerticesw1[i] =  composite* posicionI* wheelrotate*  scale2*  tempw1;
            //newVerticesw1[i] =  wheelrotate *  newVerticesw1[i];
        }
        for (int i=0; i<newVerticesw2.Length; i++)
        {
            Vector4 tempw2 = new Vector4(baseVerticesw2[i].x, baseVerticesw2[i].y, baseVerticesw2[i].z, 1);
            newVerticesw2[i] = composite *posicionD * wheelrotate* scale2 *  tempw2;
        }
        for (int i=0; i<newVerticesw3.Length; i++)
        {
            Vector4 tempw3 = new Vector4(baseVerticesw3[i].x, baseVerticesw3[i].y, baseVerticesw3[i].z, 1);
            newVerticesw3[i] = composite *posicionIat * wheelrotate* scale2 *  tempw3;
        }
        for (int i=0; i<newVerticesw4.Length; i++)
        {
            Vector4 tempw4 = new Vector4(baseVerticesw4[i].x, baseVerticesw4[i].y, baseVerticesw4[i].z, 1);
            newVerticesw4[i] = composite *posicionDat * wheelrotate* scale2 *  tempw4;
        }
        //apply the matrices to the vertices
        mesh.vertices = newVertices;
        wheel1.vertices = newVerticesw1;
        wheel2.vertices = newVerticesw2;
        wheel3.vertices = newVerticesw3;
        wheel4.vertices = newVerticesw4;
        mesh.RecalculateNormals();
        wheel1.RecalculateNormals();
        wheel2.RecalculateNormals();
        wheel3.RecalculateNormals();
        wheel4.RecalculateNormals();

    
    }

    Vector3 PositionLerp(Vector3 start, Vector3 end, float time)
    {
        return start + (end - start) * time;
    }

    float getT(){
        currentTime+=Time.deltaTime;
        T=currentTime/motionTime;
        if(T>1){
            T=1;
        }
        return T;
    }

    public getPosition(){
        //swap variables de donde estas y a donde vas.

    }

}
