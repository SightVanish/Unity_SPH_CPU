using UnityEngine;
using System.Collections.Generic;
using System.IO;
using Matrix = MathNet.Numerics.LinearAlgebra.Matrix<double>;
using MathNet.Numerics.LinearAlgebra.Double;
using System;
using System.Threading;
using System.Linq;
using System.Threading.Tasks;

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
    private Matrix M; // moment matrix

    private List<Particle> neighbours;
    private float elpasedTime = 0.0f;
    private float extraTime = 0.0f;
    [Header("Rendering")]
    [SerializeField]
    [Tooltip("Rendering time interval.")]
    private float timeStep = 0.05f;
    private float timeStep2;
    private int numchunks; // render #numchunks for each frame

    // parallel
    private Vector3 currentPosition;
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
        currentPosition = Vector3.zero;

        timeStep2 = timeStep * timeStep;
        minDist2 = minDist * Manager.Radius * minDist * Manager.Radius;
        elpasedTime = 0.0f;
        extraTime = 0.0f;
        POLY6 = 315.0f / (64.0f * Mathf.PI * Mathf.Pow(h, 9));
        SPIKY = -45.0f / (Mathf.PI * Mathf.Pow(h, 6));
        M = new DenseMatrix(4, 4);
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
        GetNeighbourParticles();

        for (int i = 0; i < numchunks; i++)
        {
            UpdatePositions();
        }

        /*
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
        */

    }

    // update particle position
    private void UpdatePositions()
    {
        f_pressure = f_viscosity = Vector3.zero;

        // density
        if (Manager.MLS)
        {
            // TODO: what if M is not invertible
            M = MomentMatrix_Optimized();
            if (M.Determinant() != 0)
            {
                density = 0.0f;
                density =
                neighbours.AsParallel().Sum(x =>
                {
                    return mass * PHI_Optimized_Parallel(x.currentPosition);
                });
                /*
                foreach (var nei in neighbours)
                {
                    density += nei.mass * PHI_Optimized(nei.gameObject);
                }
                */
            }
            else
                Debug.Log("M is not invertible");
        }
        else
        {
            density = 0.0f;
            density =
                neighbours.AsParallel().Sum(x =>
                {
                    return mass * Wploy6_parallel(x.currentPosition);
                });
                
            /*
            foreach (var nei in neighbours)
            {
                // TODO: how to compute density, if there is only this in neighbours
                density += nei.mass * Wploy6(nei.gameObject);
            }
            if (Mathf.Abs(density1 - density) > 1e5)
                Debug.Log("density parallel failed.");
            */
        }

        //// DEBUG
        //if (this.gameObject.name == "Particle0")
        //{
        //    using (StreamWriter sw = new StreamWriter("log.txt"))
        //    {
        //        sw.WriteLine(this.gameObject.name);
        //        sw.WriteLine(" ");
        //        sw.WriteLine(density);
        //    }
        //}
        if (density==0)
            print("density is 0.");


        // pressure
        // TODO: the result may be negative
        pressure = K * (density - RestDensity);

        // force
        // TODO: parallel
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
            acc += f_pressure / density;

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


        SimpleCollision();
        // ComputeCollision();
        currentPosition = transform.position;
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

    private float Wploy6_parallel(Vector3 particle)
    {
        Vector3 dist = currentPosition - particle;
        float r2 = Vector3.Dot(dist, dist);

        if (r2 >= h * h)
            return 0;
        else
            return POLY6 *Mathf.Pow(h * h - r2, 3);
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

    #region MLS
    private float PHI(GameObject particle)
    {
        // order = 1
        Matrix p = new DenseMatrix(4, 1);
        Matrix p_i = new DenseMatrix(4, 1);
        // p = [1, x, y, z]^T
        p[0, 0] = 1f;
        p[1, 0] = transform.position.x;
        p[2, 0] = transform.position.y;
        p[3, 0] = transform.position.z;
        p_i[0, 0] = 1f;
        p_i[1, 0] = particle.transform.position.x;
        p_i[2, 0] = particle.transform.position.y;
        p_i[3, 0] = particle.transform.position.z;

        var phi = Wploy6(particle) * p.Transpose() * M.Inverse() * p_i;

        return (float)phi[0, 0];
    }
    private Matrix MomentMatrix()
    {
        Matrix M = new DenseMatrix(4, 4);
        Matrix p = new DenseMatrix(4, 1);
        foreach (var nei in neighbours)
        {
            p[0, 0] = 1f;
            p[1, 0] = nei.transform.position.x;
            p[2, 0] = nei.transform.position.y;
            p[3, 0] = nei.transform.position.z;
            M += Wploy6(nei.gameObject) * p * p.Transpose();
        }
        return M;
    }
    private float PHI_Optimized(GameObject particle)
    {
        // order = 1
        Matrix p = new DenseMatrix(4, 1);
        Matrix p_i = new DenseMatrix(4, 1);
        // p = [1, x, y, z]^T
        p[0, 0] = 1f; p[1, 0] = 0; p[2, 0] = 0; p[3, 0] = 0;
        p_i[0, 0] = 1f;
        p_i[1, 0] = (particle.transform.position.x - transform.position.x) / h;
        p_i[2, 0] = (particle.transform.position.y - transform.position.y) / h;
        p_i[3, 0] = (particle.transform.position.z - transform.position.z) / h;

        var phi = Wploy6(particle) * p.Transpose() * M.Inverse() * p_i;

        return (float)phi[0, 0];
    }
    private float PHI_Optimized_Parallel(Vector3 particle)
    {
        // order = 1
        Matrix p = new DenseMatrix(4, 1);
        Matrix p_i = new DenseMatrix(4, 1);
        // p = [1, x, y, z]^T
        p[0, 0] = 1f; p[1, 0] = 0; p[2, 0] = 0; p[3, 0] = 0;
        p_i[0, 0] = 1f;
        p_i[1, 0] = (particle.x - currentPosition.x) / h;
        p_i[2, 0] = (particle.y - currentPosition.y) / h;
        p_i[3, 0] = (particle.z - currentPosition.z) / h;

        var phi = Wploy6_parallel(particle) * p.Transpose() * M.Inverse() * p_i;

        return (float)phi[0, 0];
    }
    private Matrix MomentMatrix_Optimized()
    {
        Matrix M = new DenseMatrix(4, 4);
        Matrix p = new DenseMatrix(4, 1);
        // TODO: parallel
        foreach (var nei in neighbours)
        {
            p[0, 0] = 1f;
            p[1, 0] = (nei.transform.position.x - transform.position.x) / h;
            p[2, 0] = (nei.transform.position.y - transform.position.y) / h;
            p[3, 0] = (nei.transform.position.z - transform.position.z) / h;
            M += Wploy6(nei.gameObject) * p * p.Transpose();
        }
        return M;
    }

    #endregion

    #region NeighbourSearch
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
    private void GetNeighbourParticles()
    {
        neighbours = new List<Particle>();
        // only consider 6 nearest grid cube
        AddParticle(gridPos, ref neighbours);
        AddParticle(new Vector3(gridPos.x + 1, gridPos.y, gridPos.z), ref neighbours); // right
        AddParticle(new Vector3(gridPos.x - 1, gridPos.y, gridPos.z), ref neighbours); // left
        AddParticle(new Vector3(gridPos.x, gridPos.y, gridPos.z + 1), ref neighbours); // front
        AddParticle(new Vector3(gridPos.x, gridPos.y, gridPos.z - 1), ref neighbours); // back
        AddParticle(new Vector3(gridPos.x, gridPos.y + 1, gridPos.z), ref neighbours); // up
        AddParticle(new Vector3(gridPos.x, gridPos.y - 1, gridPos.z), ref neighbours); // down

    }

    // add the particles in Manager.hashGrid[pos] to lp
    private void AddParticle(Vector3 pos, ref List<Particle> lp)
    {
        if (Manager.hashGrid.ContainsKey(pos))
        {
            foreach (var particle in Manager.hashGrid[pos])
            {
                lp.Add(particle);
            }
        }
    }

    #endregion


    #region Collision
    /*
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
    */

    private void SimpleCollision()
    {
        // TODO:
        float collisionDamp = 0f;
        // handle collision
        if (transform.position.y < 0)
        {
            lastPosition = transform.position;
            transform.position = new Vector3(transform.position.x, -collisionDamp * transform.position.y, transform.position.z);
        }
        //if (transform.position.y > 100f)
        //{
        //    transform.position = new Vector3(transform.position.x, 100f - collisionDamp * (transform.position.x - 30f), transform.position.z);
        //}
        if (transform.position.x < -15f)
        {
            lastPosition = transform.position;
            transform.position = new Vector3(-15f - collisionDamp * (transform.position.x - (-15f)), transform.position.y, transform.position.z);
        }
        if (transform.position.x > 15f)
        {
            lastPosition = transform.position;
            transform.position = new Vector3(15f - collisionDamp * (transform.position.x - 15f), transform.position.y, transform.position.z);
        }
        if (transform.position.z < -15f)
        {
            lastPosition = transform.position;
            transform.position = new Vector3(transform.position.x, transform.position.y, -15f - collisionDamp * (transform.position.z - (-15f)));
        }
        if (transform.position.z > 15f)
        {
            lastPosition = transform.position;
            transform.position = new Vector3(transform.position.x, transform.position.y, 15f - collisionDamp * (transform.position.z - 15f));
        }
    }

    private void ComputeCollision()
    {
        foreach (var collider in Manager.collisions)
        {
            float penetrationLength;
            if (Intersection(collider, out penetrationLength))
            {
                lastPosition = transform.position;
                transform.position = transform.position - collider.penetrationNormal * Mathf.Abs(penetrationLength);
            }
        }
    }

    private bool Intersection(SPHCollision collider, out float penetrationLength)
    {
        Vector3 colliderProjection = collider.position - transform.position;
        penetrationLength = Mathf.Abs(Vector3.Dot(colliderProjection, collider.penetrationNormal)) - (Manager.Radius / 2.0f);

        return penetrationLength < 0.0f
            && Mathf.Abs(Vector3.Dot(colliderProjection, collider.right)) < collider.scale.x
            && Mathf.Abs(Vector3.Dot(colliderProjection, collider.up)) < collider.scale.y;
    }
    #endregion
}
