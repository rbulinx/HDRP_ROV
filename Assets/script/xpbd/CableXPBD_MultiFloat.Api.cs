using UnityEngine;

public partial class CableXPBD_MultiFloat
{
    public int GetNodeCount()
    {
        EnsureInitialized(force: false);
        return x != null ? x.Length : Mathf.Max(2, nodeCount);
    }

    public Vector3 GetNodePosition(int i)
    {
        EnsureInitialized(force: false);

        if (x == null || x.Length == 0)
            return (topAnchor ? topAnchor.position : transform.position);

        Vector3 p;
        if (i <= 0) p = x[0];
        else if (i >= x.Length) p = x[x.Length - 1];
        else p = x[i];

        if (IsFinite(p))
            return p;

        if (topAnchor != null && IsFinite(topAnchor.position))
            return topAnchor.position;

        if (IsFinite(transform.position))
            return transform.position;

        return Vector3.zero;
    }

    public float GetCableLengthMeters() => lastCableLength;
    public float GetTensionNewton() => lastTensionN;
    public float GetCurrentTensionNewton() => lastCurrentTensionN;
    public float GetCableBuoyancyLoadNewton() => lastCableBuoyancyLoadN;
    public float GetFloatLoadNewton() => floatLoadOnBottom.magnitude;
    public int GetFloatCount() => floatInstances.Count;
    public float GetBottomSegmentTensionNewton() => lastBottomSegmentTensionN;
    public float GetStretchMeters() => lastStretch;
    public string GetHydrodynamicDragModelName() => hydrodynamicDragModel.ToString();

    public void SetHydrodynamicDragModel(HydrodynamicDragModel model)
    {
        hydrodynamicDragModel = model;
    }

    public void ToggleHydrodynamicDragModel()
    {
        hydrodynamicDragModel = hydrodynamicDragModel == HydrodynamicDragModel.LegacyCoefficients
            ? HydrodynamicDragModel.Morison
            : HydrodynamicDragModel.LegacyCoefficients;
    }

    public void ReelOutStep()
    {
        SetTargetDeployedLength(targetDeployedLength + winchStepMeters);
    }

    public void ReelInStep()
    {
        SetTargetDeployedLength(targetDeployedLength - winchStepMeters);
    }

    public void SetTargetDeployedLength(float lengthMeters)
    {
        targetDeployedLength = Mathf.Clamp(lengthMeters, minLength, maxLength);
    }

    void OnValidate()
    {
        nodeCount = Mathf.Max(2, nodeCount);
        minLength = Mathf.Max(0.01f, minLength);
        maxLength = Mathf.Max(minLength, maxLength);

        deployedLength = Mathf.Clamp(deployedLength, minLength, maxLength);
        targetDeployedLength = Mathf.Clamp(targetDeployedLength, minLength, maxLength);

        massPerNode = Mathf.Max(1e-4f, massPerNode);
        nodeRadius = Mathf.Max(1e-4f, nodeRadius);
        targetSegmentLength = Mathf.Max(0.05f, targetSegmentLength);
        ApplyAutoNodeCountForLength();

        solverIterations = Mathf.Clamp(solverIterations, 1, 80);
        substeps = Mathf.Clamp(substeps, 1, 12);
        maxOverlapsPerNode = Mathf.Clamp(maxOverlapsPerNode, 1, 32);
        collisionIterationStride = Mathf.Clamp(collisionIterationStride, 1, 8);
        ignoreCollisionSegmentsNearTopAnchor = Mathf.Max(0, ignoreCollisionSegmentsNearTopAnchor);
        ignoreCollisionSegmentsNearBottomAttach = Mathf.Max(0, ignoreCollisionSegmentsNearBottomAttach);

        axialRigidityEA = Mathf.Max(0f, axialRigidityEA);
        bendingRigidityEI = Mathf.Max(0f, bendingRigidityEI);

        distanceCompliance = Mathf.Max(0f, distanceCompliance);
        slackMeters = Mathf.Max(0f, slackMeters);
        requireNearTautMeters = Mathf.Max(0f, requireNearTautMeters);

        axialDamping = Mathf.Max(0f, axialDamping);
        maxTensionNewton = Mathf.Max(0f, maxTensionNewton);
        maxTensionRate = Mathf.Max(0f, maxTensionRate);
        maxBottomSegmentConstraintTensionNewton = Mathf.Max(0f, maxBottomSegmentConstraintTensionNewton);

        tensionSmoothingHz = Mathf.Max(0.1f, tensionSmoothingHz);
        directionSmoothingHz = Mathf.Max(0.1f, directionSmoothingHz);
        bendingMaxCorrection = Mathf.Max(0f, bendingMaxCorrection);

        maxCollisionCorrection = Mathf.Max(0f, maxCollisionCorrection);
        collisionVelocityDamping = Mathf.Clamp01(collisionVelocityDamping);
        dragLinearAlong = Mathf.Max(0f, dragLinearAlong);
        dragLinearAcross = Mathf.Max(0f, dragLinearAcross);
        dragQuadraticAlong = Mathf.Max(0f, dragQuadraticAlong);
        dragQuadraticAcross = Mathf.Max(0f, dragQuadraticAcross);
        morisonWaterDensity = Mathf.Max(0f, morisonWaterDensity);
        morisonCableDiameter = Mathf.Max(1e-5f, morisonCableDiameter);
        morisonNormalDragCoefficient = Mathf.Max(0f, morisonNormalDragCoefficient);
        morisonTangentialDragCoefficient = Mathf.Max(0f, morisonTangentialDragCoefficient);
        morisonDragScale = Mathf.Max(0f, morisonDragScale);
        maxHydrodynamicAcceleration = Mathf.Max(0f, maxHydrodynamicAcceleration);
        maxCableNodeSpeed = Mathf.Max(0f, maxCableNodeSpeed);
        maxCurrentTensionNewton = Mathf.Max(0f, maxCurrentTensionNewton);
        maxCableBuoyancyLoadNewton = Mathf.Max(0f, maxCableBuoyancyLoadNewton);
        inspectionCableLineWidth = Mathf.Max(0.001f, inspectionCableLineWidth);
        floatSonarColliderScale = Mathf.Max(0.01f, floatSonarColliderScale);
        cableSonarRadius = Mathf.Max(0.001f, cableSonarRadius);
        cableSonarEndOverlap = Mathf.Max(0f, cableSonarEndOverlap);
        sonarColliderUpdateStride = Mathf.Clamp(sonarColliderUpdateStride, 1, 12);
        floatWaterDensity = Mathf.Max(0f, floatWaterDensity);
        floatGravity = Mathf.Max(0f, floatGravity);
        maxFloatAcceleration = Mathf.Max(0f, maxFloatAcceleration);
        maxFloatForce = Mathf.Max(0f, maxFloatForce);
        floatForceSmoothing = Mathf.Clamp01(floatForceSmoothing);
        floatCollisionRadiusScale = Mathf.Max(0.01f, floatCollisionRadiusScale);
        maxFloatCollisionCorrection = Mathf.Max(0f, maxFloatCollisionCorrection);
        winchStepMeters = Mathf.Max(0.1f, winchStepMeters);

        if (floatSections != null)
        {
            for (int i = 0; i < floatSections.Count; i++)
            {
                FloatSection section = floatSections[i];
                if (section == null)
                    continue;

                section.startNormalized = Mathf.Clamp01(section.startNormalized);
                section.endNormalized = Mathf.Clamp01(section.endNormalized);
                section.spacingMeters = Mathf.Max(0.01f, section.spacingMeters);
                section.diameter = Mathf.Max(0.001f, section.diameter);
                section.length = Mathf.Max(0.001f, section.length);
                section.massKg = Mathf.Max(0f, section.massKg);
                section.buoyancyScale = Mathf.Max(0f, section.buoyancyScale);
                section.normalDragCd = Mathf.Max(0f, section.normalDragCd);
                section.axialDragCd = Mathf.Max(0f, section.axialDragCd);
                section.normalAddedMassCa = Mathf.Max(0f, section.normalAddedMassCa);
                section.axialAddedMassCa = Mathf.Max(0f, section.axialAddedMassCa);
                section.visualScale = Mathf.Max(0.01f, section.visualScale);
            }
        }

        floatInstancesDirty = true;

        lr = GetComponent<LineRenderer>();
        if (lr != null)
            ApplyInspectionCableLineStyleIfNeeded();

        if (winchSpeed < 0f) winchSpeed = 0f;
    }

    void OnDrawGizmosSelected()
    {
        if (!drawGizmos || x == null) return;
        float r = Mathf.Max(1e-4f, nodeRadius);
        for (int i = 0; i < x.Length; i++)
            Gizmos.DrawWireSphere(x[i], r);
    }
}
