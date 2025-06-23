using System.Collections;
using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// manages collection of flower plants and attached flowers
/// </summary>
public class FlowerArea : MonoBehaviour
{
    // the diameter of the area where agent and flowers can be
    // used for observing relative distance from agent to flower

    public const float AreaDiameter = 20f;

    //list of all flower plants in the flower area where one plants have many flowers

    private List<GameObject> flowerPlants;

    //look up dictionary for looking up  a flower from a nectar collider

    private Dictionary<Collider, Flower> nectarFlowerDict;


    /// <summary>
    /// the list of all flowers in the flower area (plant)
    /// </summary>
    public List<Flower> Flowers { get; private set; }


    public void ResetFlowers()
    {
        //rotate each flower plant around the Y axis and little around X and Z axis
        foreach (GameObject flowerPlant in flowerPlants)
        {
            float xRotation = UnityEngine.Random.Range(-5f, 5f);
            float zRotation = UnityEngine.Random.Range(-5f, 5f);
            float yRotation = UnityEngine.Random.Range(-180f, 180f);
            flowerPlant.transform.localRotation = Quaternion.Euler(xRotation, yRotation, zRotation);


        }

        //Reset each flower
        foreach (Flower flower in Flowers)
        {
            flower.ResetFlower();

        }
    }

    /// <summary>
    /// get the <see cref="Flower "/> that a nectar collider belongs to 
    /// </summary>
    /// <param name="collider">nectar collider </param>
    /// <returns> matching flower </returns>
    public Flower GetFlowerFromNectar(Collider collider)

    {
        return nectarFlowerDict[collider];
    }

    /// <summary>
    /// called on awake of the area
    /// </summary>
    private void Awake()
    {
        //Intialize variables
        flowerPlants = new List<GameObject>();
        nectarFlowerDict = new Dictionary<Collider, Flower>();
        Flowers = new List<Flower>();

    }

    private void Start()
    {
        //find all flowers that are children of the this gameobject
        FindChildFlowers(transform);
    }

    
    /// <summary>
    /// Recursivly finds all flowers and flower plants  that are children of a parent transform
    /// </summary>
    /// <param name="parent"> the parent of the child to check </param>
    private void FindChildFlowers(Transform parent)
    {
        for (int i = 0; i < parent.childCount; i++)
        {
            Transform child = parent.GetChild(i);

            if (child.CompareTag("flower_plant"))
            {
                //if found a flower plant then add it to the flowerplants
                flowerPlants.Add(child.gameObject);

                //Look for flowers within the flower plant
                FindChildFlowers(child);
            }
            else
            {
                //if not a flower plant then dont look for a flower component
                Flower flower = child.GetComponent<Flower>();
                if (flower != null)
                {
                    //if found a flower add it to the flowers list
                    Flowers.Add(flower);

                    //add the nectar collider to the lookup dict
                    nectarFlowerDict.Add(flower.nectarCollider, flower);

                }
                else
                {
                    //if flower component not found then check children
                    FindChildFlowers(child);
                }
            }
        }
    }
}
