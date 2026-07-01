using UnityEngine;

namespace RbulinX.SitlEngine
{
    /// <summary>
    /// Vehicle-specific physics plugged into SitlEngineCore. The core handles
    /// transport/telemetry/coordinate frames; everything about how the body
    /// actually moves (buoyancy, rotor lift, drag, thruster mixing, ...) lives here.
    /// </summary>
    public interface IVehicleForceModule
    {
        void OnSitlEnable(SitlEngineCore core, Rigidbody rb, Transform vehicleTransform);
        void OnSitlDisable();

        /// <summary>Called once per FixedUpdate, after the core applies mass/damping.</summary>
        void ApplyForces(SitlEngineCore core, Rigidbody rb, Transform vehicleTransform, float fixedDeltaTime, bool actuatorOutputIsFresh);

        /// <summary>
        /// Lets the module supply a vertical reference (e.g. ROV water surface Y) used
        /// as the origin for reported position. Return false to use world zero.
        /// </summary>
        bool TryGetVerticalOriginY(Vector3 worldPosition, out float originY);
    }
}
