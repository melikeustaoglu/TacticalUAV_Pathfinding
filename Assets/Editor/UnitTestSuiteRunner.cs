using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Reflection;
using NUnit.Framework;
using UnityEditor;
using UnityEngine;
using Debug = UnityEngine.Debug;

/// <summary>
/// Automated Test Suite Runner (Editor-Only).
/// Reflectively executes all EditMode NUnit unit tests in the project,
/// verifying system invariants and printing a detailed execution report.
/// </summary>
public static class UnitTestSuiteRunner
{
    public struct TestExecutionResult
    {
        public string FixtureName;
        public string TestName;
        public bool Passed;
        public string ErrorMessage;
        public double ElapsedMilliseconds;
    }

    [MenuItem("Tactical UAV/Run All Unit Tests (EditMode)", priority = 20)]
    public static void RunAllEditModeTests()
    {
        Debug.Log("<b>[UnitTestSuiteRunner]</b> Initiating EditMode Unit Test Suite execution...");

        List<TestExecutionResult> results = new List<TestExecutionResult>();
        Assembly[] assemblies = AppDomain.CurrentDomain.GetAssemblies();

        int passedCount = 0;
        int failedCount = 0;
        Stopwatch totalTimer = Stopwatch.StartNew();

        foreach (Assembly assembly in assemblies)
        {
            string assemblyName = assembly.GetName().Name;
            if (!assemblyName.Contains("Editor") && !assemblyName.Contains("Test") && !assemblyName.Contains("Assembly-CSharp"))
                continue;

            Type[] types;
            try
            {
                types = assembly.GetTypes();
            }
            catch (ReflectionTypeLoadException ex)
            {
                types = ex.Types;
            }

            if (types == null)
                continue;

            foreach (Type type in types)
            {
                if (type == null || !type.IsClass || type.IsAbstract)
                    continue;

                // Check for [TestFixture] attribute
                if (type.GetCustomAttribute<TestFixtureAttribute>() == null && !type.Name.EndsWith("Tests"))
                    continue;

                MethodInfo setUpMethod = null;
                MethodInfo tearDownMethod = null;

                foreach (MethodInfo method in type.GetMethods(BindingFlags.Public | BindingFlags.NonPublic | BindingFlags.Instance))
                {
                    if (method.GetCustomAttribute<SetUpAttribute>() != null)
                        setUpMethod = method;
                    if (method.GetCustomAttribute<TearDownAttribute>() != null)
                        tearDownMethod = method;
                }

                foreach (MethodInfo testMethod in type.GetMethods(BindingFlags.Public | BindingFlags.NonPublic | BindingFlags.Instance))
                {
                    if (testMethod.GetCustomAttribute<TestAttribute>() == null)
                        continue;

                    object instance = null;
                    try
                    {
                        instance = Activator.CreateInstance(type);
                    }
                    catch (Exception ex)
                    {
                        Debug.LogError($"[UnitTestSuiteRunner] Failed to instantiate fixture '{type.Name}': {ex.Message}");
                        continue;
                    }

                    Stopwatch testTimer = Stopwatch.StartNew();
                    bool testPassed = true;
                    string error = null;

                    try
                    {
                        setUpMethod?.Invoke(instance, null);
                        testMethod.Invoke(instance, null);
                    }
                    catch (TargetInvocationException ex)
                    {
                        testPassed = false;
                        error = ex.InnerException != null ? ex.InnerException.Message : ex.Message;
                    }
                    catch (Exception ex)
                    {
                        testPassed = false;
                        error = ex.Message;
                    }
                    finally
                    {
                        try
                        {
                            tearDownMethod?.Invoke(instance, null);
                        }
                        catch (Exception tearDownEx)
                        {
                            Debug.LogWarning($"[UnitTestSuiteRunner] TearDown error in '{type.Name}': {tearDownEx.Message}");
                        }
                        testTimer.Stop();
                    }

                    if (testPassed)
                    {
                        passedCount++;
                    }
                    else
                    {
                        failedCount++;
                        Debug.LogError($"<b>[FAIL]</b> {type.Name}.{testMethod.Name}: {error}");
                    }

                    results.Add(new TestExecutionResult
                    {
                        FixtureName = type.Name,
                        TestName = testMethod.Name,
                        Passed = testPassed,
                        ErrorMessage = error,
                        ElapsedMilliseconds = testTimer.Elapsed.TotalMilliseconds
                    });
                }
            }
        }

        totalTimer.Stop();

        // Print Summary Report
        int totalTests = passedCount + failedCount;
        string statusColor = failedCount == 0 ? "#4CAF50" : "#F44336";
        string header = $"<color={statusColor}><b>[UnitTestSuiteRunner Summary] Total: {totalTests} | Passed: {passedCount} | Failed: {failedCount} (Elapsed: {totalTimer.ElapsedMilliseconds} ms)</b></color>";

        Debug.Log(header);

        if (failedCount == 0)
        {
            Debug.Log($"<b><color=#4CAF50>✔ ALL {totalTests} EDITMODE UNIT TESTS PASSED SUCCESSFULLY!</color></b>");
        }
        else
        {
            Debug.LogWarning($"<b><color=#F44336>✘ {failedCount} TESTS FAILED. Please inspect console errors above.</color></b>");
        }
    }
}
