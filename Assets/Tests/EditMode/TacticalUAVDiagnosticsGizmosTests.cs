using System;
using System.Collections.Generic;
using NUnit.Framework;
using UnityEngine;

/// <summary>
/// Unit and integration tests for TacticalUAVDiagnosticsGizmos 3D Scene View visualization layer.
/// Validates safe null-handling, reference acquisition, toggle configurability, and non-intrusive runtime integration.
/// </summary>
[TestFixture]
public class TacticalUAVDiagnosticsGizmosTests
{
    private GameObject uavObj;
    private TacticalUAVDiagnosticsGizmos gizmos;

    [SetUp]
    public void SetUp()
    {
        uavObj = new GameObject("DiagnosticsTestUAV");
        gizmos = uavObj.AddComponent<TacticalUAVDiagnosticsGizmos>();
    }

    [TearDown]
    public void TearDown()
    {
        if (uavObj != null)
        {
            UnityEngine.Object.DestroyImmediate(uavObj);
        }
    }

    [Test]
    public void DiagnosticsGizmos_DefaultsAllFeatureTogglesToTrue()
    {
        Assert.IsTrue(gizmos.ShowDiagnostics, "Master showDiagnostics toggle must default to true!");
        Assert.IsTrue(gizmos.ShowLidar, "showLidar toggle must default to true!");
        Assert.IsTrue(gizmos.ShowLidarHits, "showLidarHits toggle must default to true!");
        Assert.IsTrue(gizmos.ShowRadar, "showRadar toggle must default to true!");
        Assert.IsTrue(gizmos.ShowRadarDetections, "showRadarDetections toggle must default to true!");
        Assert.IsTrue(gizmos.ShowThreats, "showThreats toggle must default to true!");
        Assert.IsTrue(gizmos.ShowSafetyEnvelope, "showSafetyEnvelope toggle must default to true!");
        Assert.IsTrue(gizmos.ShowPredictedCollision, "showPredictedCollision toggle must default to true!");
        Assert.IsTrue(gizmos.ShowVelocityObstacle, "showVelocityObstacle toggle must default to true!");
        Assert.IsTrue(gizmos.ShowEkfUncertainty, "showEkfUncertainty toggle must default to true!");
        Assert.IsTrue(gizmos.ShowTracks, "showTracks toggle must default to true!");
        Assert.IsTrue(gizmos.ShowTrackPredictions, "showTrackPredictions toggle must default to true!");
        Assert.IsTrue(gizmos.ShowPath, "showPath toggle must default to true!");
    }

    [Test]
    public void DiagnosticsGizmos_AllowsDynamicTogglingOfFeatures()
    {
        gizmos.ShowDiagnostics = false;
        gizmos.ShowLidar = false;
        gizmos.ShowRadar = false;
        gizmos.ShowThreats = false;
        gizmos.ShowEkfUncertainty = false;
        gizmos.ShowTracks = false;
        gizmos.ShowPath = false;

        Assert.IsFalse(gizmos.ShowDiagnostics);
        Assert.IsFalse(gizmos.ShowLidar);
        Assert.IsFalse(gizmos.ShowRadar);
        Assert.IsFalse(gizmos.ShowThreats);
        Assert.IsFalse(gizmos.ShowEkfUncertainty);
        Assert.IsFalse(gizmos.ShowTracks);
        Assert.IsFalse(gizmos.ShowPath);
    }

    [Test]
    public void DiagnosticsGizmos_DoesNotThrowWhenOptionalSystemsAreAbsent()
    {
        // Bare GameObject with only TacticalUAVDiagnosticsGizmos
        Assert.DoesNotThrow(() =>
        {
            gizmos.AcquireReferences();
        });
    }

    [Test]
    public void DiagnosticsGizmos_AcquiresAllAttachedAutonomyReferencesSafely()
    {
        Vector3 spawnPos = new Vector3(5f, 2f, 5f);
        GameObject runtimeUav = GameManagerBootstrapper.CreateUav(spawnPos);

        TacticalUAVDiagnosticsGizmos diag = runtimeUav.GetComponent<TacticalUAVDiagnosticsGizmos>();
        Assert.IsNotNull(diag, "GameManagerBootstrapper.CreateUav must attach TacticalUAVDiagnosticsGizmos!");

        Assert.DoesNotThrow(() =>
        {
            diag.AcquireReferences();
        });

        UnityEngine.Object.DestroyImmediate(runtimeUav);
    }
}
