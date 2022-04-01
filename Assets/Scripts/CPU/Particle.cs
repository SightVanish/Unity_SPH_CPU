using UnityEngine;
using System.Collections.Generic;

public class Particle : MonoBehaviour
{
    #region Parameters
    public SPH Manager;
    [Range(1, 4)]
    [Tooltip("Increase HashGridScale to reduce the grid cube size.")]
    public int HashGridScale = 1;
    public float mass = 1.0f;
    [Header("Wpoly6")]
    public float h = 2.0f;
    [Header("Gas constant")]
    public float K = 10.0f;
    public float RestDensity = 0.5f;
    [Header("Damping")]
    public float velocityDamping = 0.98f;
    [Header("Replusive")]
    public float minDist = 0.5f; // the disttance that will cause replusive force
    private float minDist2;
    [HideInInspector]
    public Vector3 lastPosition;
    [HideInInspector]
    public Vector3 position;
    [HideInInspector]
    public Vector3 acc;
    [HideInInspector]
    public Vector3 velocity;
    [HideInInspector]
    public float density;
    [HideInInspector]
    public float pressure;
    [HideInInspector]
    public Vector3 gridPos;

    private Vector3 f_pressure;
    private Vector3 f_viscosity;
    private Vector3 replusiveOffset;

    private float POLY6;
    private float SPIKY;

    private List<Particle> neighbours;
    private float elpasedTime = 0.0f;
    private float extraTime = 0.0f;
    [Header("Rendering")]
    [SerializeField]
    [Tooltip("Rendering time interval.")]
    private float timeStep = 0.05f;
    private float timeStep2;
    private int numchunks; // render #numchunks for each frame
    #endregion

    private void Start()
    {
        Time.fixedDeltaTime = 0.01f;
        InitParameters();
    }

    private void InitParameters()
    {
        Manager = GameObject.FindGameObjectWithTag("Manager").GetComponent<SPH>();
        lastPosition = position = transform.position;
        acc = velocity = Vector3.zero;

        timeStep2 = timeStep * timeStep;
        minDist2 = minDist * Manager.Radius * minDist * Manager.Radius;
        elpasedTime = 0.0f;
        extraTime = 0.0f;
        POLY6 = 315.0f / (64.0f * Mathf.PI * Mathf.Pow(h, 9));
        SPIKY = -45.0f / (Mathf.PI * Mathf.Pow(h, 6));
    }

    private void FixedUpdate()
    {
        // elpasedTime
        elpasedTime = Time.deltaTime;
        elpasedTime += extraTime;

        numchunks = (int)Mathf.Floor(elpasedTime / timeStep);
        extraTime = elpasedTime - timeStep * numchunks;

        // neighbour search
        SpatialHash();
        neighbours = GetNeighbourParticles(gridPos);

        for (int i = 0; i < numchunks; i++)
        {
            UpdatePositions();
        }

        // add repulsion and attraction
        // TODO: add attraction
        replusiveOffset = Vector3.zero;
        foreach (var nei in neighbours)
        {
            if (nei == this)
                continue;
            Vector3 dist = transform.position - nei.transform.position;
            // if two particles are too close, add repulsion
            if (dist.sqrMagnitude < minDist2)
                replusiveOffset += 0.5f * dist.normalized * (minDist2 - dist.sqrMagnitude);
        }
        transform.position += replusiveOffset;

    }

    // update particle position
    private void UpdatePositions()
    {
        density = 0.0f;
        f_pressure = f_viscosity = Vector3.zero;

        foreach (var nei in neighbours)
        {
            // TODO: how to compute density, if there is only this in neighbours
            density += nei.mass * Wploy6(nei.gameObject);
        }

        // pressure
        // TODO: the result may be negative
        pressure = K * (density - RestDensity);

        // force
        foreach (var nei in neighbours)
        {
            // if nei == this, the value calculated is 0
            f_pressure -= nei.mass * d_Wspiky(nei.gameObject) * (nei.pressure + pressure) / (2 * nei.density);
            f_viscosity += nei.mass * Wvis(nei.gameObject) * (nei.velocity - velocity) / nei.density;
        }

        // compute acceleration
        // gravity acc
        acc = Manager.gravity;

        // pressure acc
        if (!float.IsNaN(f_pressure.x) && !float.IsNaN(f_pressure.y) && !float.IsNaN(f_pressure.z) &&
            !float.IsInfinity(f_pressure.x) && !float.IsInfinity(f_pressure.y) && !float.IsInfinity(f_pressure.z))
        {
            acc += f_pressure / density;

            //if (float.IsNaN(density))
                //print(density);

        }


        // viscosity
        if (!float.IsNaN(f_viscosity.x) && !float.IsNaN(f_viscosity.y) && !float.IsNaN(f_viscosity.z) &&
            !float.IsInfinity(f_viscosity.x) && !float.IsInfinity(f_viscosity.y) && !float.IsInfinity(f_viscosity.z))
            acc += f_viscosity / density;

        velocity = transform.position - lastPosition;

        // damping velocity
        velocity *= velocityDamping;

        // update position

        position = transform.position + velocity + acc * timeStep2;
        lastPosition = transform.position;
        transform.position = position;

        acc = Vector3.zero;

        // TODO:
        // handle collision
        if (transform.position.y < 0)
        {
            //transform.position = new Vector3(transform.position.x, 0, transform.position.z);
            transform.position = new Vector3(transform.position.x, -0.25f * transform.position.y, transform.position.z);
        }
    }

    #region KernalFunction
    private float Wploy6(GameObject particle)
    {
        Vector3 dist = transform.position - particle.transform.position;
        float r2 = Vector3.Dot(dist, dist);

        if (r2 >= h*h)
            return 0;
        else
            return POLY6 * Mathf.Pow(h * h - r2, 3);
    }

    private Vector3 d_Wspiky(GameObject particle)
    {
        Vector3 dist = transform.position - particle.transform.position;
        float r = dist.magnitude;
        if (r >= h)
            return Vector3.zero;
        else
            return SPIKY * (h - r) * (h - r) * dist.normalized;
    }

    private float Wvis(GameObject particle)
    {
        float r = (transform.position - particle.transform.position).magnitude;
        if (r >= h)
            return 0;
        else
            return -SPIKY * (h - r);
    }
    #endregion


    #region NeighbourSeach
    // update Manager.hashGrid
    private void SpatialHash()
    {
        // remove this from hash grid
        if (Manager.hashGrid.ContainsKey(gridPos))
        {
            Manager.hashGrid[gridPos].Remove(this);
            if (Manager.hashGrid[gridPos].Count == 0)
                Manager.hashGrid.Remove(gridPos);
        }
        // (int)-0.5f = 0, (int)0.5f = 0, (int)-1.0f/2 = 0, so we need to - if x < 0
        int tx = transform.position.x < 0 ? (int)transform.position.x - 2 : (int)transform.position.x;
        int ty = transform.position.y < 0 ? (int)transform.position.y - 2 : (int)transform.position.y;
        int tz = transform.position.z < 0 ? (int)transform.position.z - 2 : (int)transform.position.z;

        gridPos.x = tx / HashGridScale; // int
        gridPos.y = ty / HashGridScale;
        gridPos.z = tz / HashGridScale;

        // add this to hashGrid
        if (!Manager.hashGrid.ContainsKey(gridPos))
            Manager.hashGrid.Add(gridPos, new List<Particle>());
        Manager.hashGrid[gridPos].Add(this);
    }

    // return the neighbour particles in pos
    private List<Particle> GetNeighbourParticles(Vector3 pos)
    {
        List<Particle> lp = new List<Particle>();
        // only consider 6 nearest grid cube
        AddParticle(pos, ref lp);
        AddParticle(new Vector3(pos.x + 1, pos.y, pos.z), ref lp); // right
        AddParticle(new Vector3(pos.x - 1, pos.y, pos.z), ref lp); // left
        AddParticle(new Vector3(pos.x, pos.y, pos.z + 1), ref lp); // front
        AddParticle(new Vector3(pos.x, pos.y, pos.z - 1), ref lp); // back
        AddParticle(new Vector3(pos.x, pos.y + 1, pos.z), ref lp); // up
        AddParticle(new Vector3(pos.x, pos.y - 1, pos.z), ref lp); // down

        return lp;
    }

    // add the particles in Manager.hashGrid[pos] to lp
    private void AddParticle(Vector3 pos, ref List<Particle> lp)
    {
        if (Manager.hashGrid.ContainsKey(pos))
        {
            foreach (var particle in Manager.hashGrid[pos])
                lp.Add(particle);
        }
    }

    #endregion

    /*
    #region Collision
    void OnCollisionEnter(Collision coll)
    {

        //print(coll.collider.name);
        if (!coll.gameObject.GetComponent<Particle>())
        {
            transform.position = coll.contacts[0].point + 0.525f * coll.contacts[0].normal.normalized;
        }

    }

    void OnCollisionStay(Collision coll)
    {

        if (!coll.gameObject.GetComponent<Particle>())
        {
            transform.position = coll.contacts[0].point + 0.525f * coll.contacts[0].normal.normalized;
        }
    }
    #endregion
    */

}
