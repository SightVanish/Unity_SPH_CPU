using UnityEngine;

namespace Fluid
{

    public struct Particle
    {
        public float mass;
        public Vector3 position;
        public Vector4 colorGradient;

        public Vector3 velocity;
        public float density;
        public float pressure;

        public Vector3 forces;

    }
}

