using System.Collections;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;

/// <summary>
/// manages a single flower with nectar
/// </summary>

public class Flower : MonoBehaviour
{
    [Tooltip("the color of the flower when full")]
    public Color fullFlowerColor = new Color(1f, 0f, 0.3f);

    [Tooltip("the color of the flower when empty")]
    public Color emptyFlowerColor = new Color(0.5f, 0f, 1f);


    /// <summary>
    /// the trigger collider representing the nectar 
    /// </summary>
    [HideInInspector]
    public Collider nectarCollider;

    //the solid collider representing the flower petals
    private Collider flowerCollider;

    // the flower's material
    private Material flowerMaterial;

    /// <summary>
    /// a vector pointing stright out of the flower
    /// </summary>
    public Vector3 FlowerUpVector
    {
        get
        {
            return nectarCollider.transform.up;
        }
    }



    /// <summary>
    /// the center position of the nectar collider
    /// </summary>
    public Vector3 FlowerCenterPosition
    {
        get
        {
            return nectarCollider.transform.position;
        }
    }


    /// <summary>
    /// the amount of nectar remaining
    /// </summary>
    public float NectarAmount { get; private set; }

    public bool HasNectar
    {
        get
        {
            return NectarAmount > 0f;
        }
    }

    /// <summary>
    /// Attempts to remove nectar from the flower
    /// </summary>
    /// <param name="amount"></param>
    /// <returns> actutal amount of nectar</returns>
    public float Feed(float amount)
    {
        //track the amount of nectar taken
        float nectarTaken = Mathf.Clamp(amount, 0f, NectarAmount);

        //Subtract the nectar
        NectarAmount -= amount;

        if (NectarAmount <= 0)
        {
            //No Nectar remaining
            NectarAmount = 0;

            //disable flower and nectar collider
            flowerCollider.gameObject.SetActive(false);
            nectarCollider.gameObject.SetActive(false);

            //Change color of empty flower
            flowerMaterial.SetColor("_BaseColor", emptyFlowerColor);


        }

        //return amount taken of nectar from flower
        return nectarTaken;
    }


    /// <summary>
    /// Resets flower
    /// </summary>
    public void ResetFlower()
    {
        //refill the nectar
        NectarAmount = 1f;

        //enable colliedrs 
        flowerCollider.gameObject.SetActive(true);
        nectarCollider.gameObject.SetActive(true);

        //change flower collor to full
        flowerMaterial.SetColor("_BaseColor", fullFlowerColor);
    }

    /// <summary>
    /// called when flower wakes up
    /// </summary>
    private void Awake()
    {
        //get flower mesh renderer and get the main material
        MeshRenderer meshRenderer = GetComponent<MeshRenderer>();
        flowerMaterial = meshRenderer.material;

        //find flower and nectar collider
        flowerCollider = transform.Find("FlowerCollider").GetComponent<Collider>();
        nectarCollider = transform.Find("FlowerNectarCollider").GetComponent<Collider>();
    }
}
