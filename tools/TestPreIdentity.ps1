param(
    [string]$SourcePath = (Join-Path $PSScriptRoot '../.reference/PhysicsRangeExtender/PhysicsRangeExtender/PhysicsRangeExtender.cs')
)
$ErrorActionPreference = 'Stop'
# Compile the actual identity methods against small API-shape stubs. This checks
# the kRPC KOSNameTag field and persisted naming fallback, not Unity physics.
$source = [IO.File]::ReadAllText((Resolve-Path -LiteralPath $SourcePath))
$start = $source.IndexOf('        private static bool HasTag(')
$end = $source.IndexOf('        private static void ApplyRangesToVessels', $start)
if ($start -lt 0 -or $end -le $start) { throw 'Identity methods not found in patched PRE source.' }
$methods = $source.Substring($start, $end - $start)
$harness = @"
using System;
using System.Linq;
using System.Collections.Generic;
public class Vessel { public string vesselName; public List<Part> parts = new List<Part>(); }
public class Part { public string tag; public List<PartModule> Modules = new List<PartModule>(); }
public class PartModule { public string moduleName; }
public class KOSNameTag : PartModule {
    public string nameTag;
    public KOSNameTag(string tag) { moduleName = "KOSNameTag"; nameTag = tag; }
}
public static class PreIdentityTests {
$methods
    private static int checks;
    private static void Check(bool value, string name) {
        checks++; if (!value) throw new Exception("Failed: " + name);
    }
    public static int Run() {
        Check(!IsManagedRecoveryVessel(null), "null vessel");
        var v = new Vessel { vesselName = "station" };
        Check(!IsManagedRecoveryVessel(v), "unrelated station");
        var p = new Part { tag = "payload" };
        v.parts.Add(p);
        Check(!IsManagedRecoveryVessel(v), "Unity tag must not grant membership");
        p.Modules.Add(new KOSNameTag("payload"));
        Check(HasTag(v, "payload"), "kRPC module tag recognized");
        Check(IsManagedRecoveryVessel(v), "payload managed");
        p.Modules.Clear();
        p.Modules.Add(new PartModule { moduleName = "KOSNameTag" });
        Check(!IsManagedRecoveryVessel(v), "missing field fails closed");
        foreach (var role in new[] { "left", "right", "core" }) {
            v = new Vessel { vesselName = "Codex booster_" + role + " d621e69ff7", parts = null };
            Check(IsManagedRecoveryVessel(v), "persistent role " + role + " without parts/tags");
        }
        foreach (var name in new[] {
            "Codex booster_left d621e69ff7 debris",
            "Codex booster_left d621e69ff7残骸",
            "Codex booster_left d621e69ff",
            "Codex booster_left d621e69fg7",
            "Codex booster_station d621e69ff7",
            "Codex booster_left d621e69ff70",
            "Codex booster_left  d621e69ff7" }) {
            Check(!IsManagedRecoveryVessel(new Vessel { vesselName = name }), "reject " + name);
        }
        return checks;
    }
}
"@
Add-Type -TypeDefinition $harness -Language CSharp
$count = [PreIdentityTests]::Run()
Write-Output "PRE identity checks passed: $count"