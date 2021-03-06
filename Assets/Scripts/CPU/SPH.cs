using System.Collections.Generic;
using UnityEngine;
using System.IO;

public class SPH : MonoBehaviour
{
    [Header("MLS")]
    public bool MLS = false;
    [Header("Respawn Particlees")]
    public GameObject Prefab;
    public float Radius = 1.0f;
    public float RespawnDensity = 1.0f;
    public int Amount = 100;
    public int PerRow = 10;
    public GameObject RespwanPosition;
    private GameObject[] particles;

    public Dictionary<Vector3, List<Particle>> hashGrid;

    public GameObject[] walls;
    public List<SPHCollision> collisions;

    [Header("Constants")]
    public Vector3 gravity = new Vector3(0.0f, -9.81f, 0.0f);

    private void Awake()
    {
        Application.targetFrameRate = 60;
        hashGrid = new Dictionary<Vector3, List<Particle>>();

        ParticleRespawn();

        //GetWalls();
        
    }
    private void ParticleRespawn()
    {
        particles = new GameObject[Amount];

        for (int i = 0; i < Amount; i++)
        {
            float x = i % PerRow + Random.Range(-0.1f, 0.1f);
            float y = i / (PerRow * PerRow) + Random.Range(-0.1f, 0.1f);
            float z = Mathf.CeilToInt(i % (PerRow * PerRow) / PerRow) + Random.Range(-0.1f, 0.1f);

            GameObject go = Instantiate(Prefab,
                new Vector3(x * RespawnDensity + RespwanPosition.transform.position.x,
                            y * RespawnDensity + RespwanPosition.transform.position.y,
                            z * RespawnDensity + RespwanPosition.transform.position.z),
                Quaternion.identity,
                RespwanPosition.transform);
            go.name = "Particle" + i;

            go.transform.localScale *= Radius;

            particles[i] = go;
        }
    }

    private void GetWalls()
    {
        walls = GameObject.FindGameObjectsWithTag("Wall");
        foreach (var w in walls)
            collisions.Add(w.GetComponent<SPHCollision>());
    }

}
