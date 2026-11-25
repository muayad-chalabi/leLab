import { useEffect, useState } from "react";
import { useApi } from "@/contexts/ApiContext";

export const SO101_URDF_PATH = "/so-101-urdf/urdf/so101_new_calib.urdf";

export interface RobotProfile {
  robot: string;
  urdf_path: string;
  bimanual: boolean;
  bimanual_urdf_path?: string | null;
}

const FALLBACK: RobotProfile = { robot: "so101", urdf_path: SO101_URDF_PATH, bimanual: false };

// One request per page load: the robot is fixed for the server's lifetime.
let cached: Promise<RobotProfile> | null = null;

function fetchRobotProfile(baseUrl: string): Promise<RobotProfile> {
  if (!cached) {
    cached = fetch(`${baseUrl}/robot-profile`)
      .then((r) => (r.ok ? r.json() : null))
      .then((data) => (data && typeof data.urdf_path === "string" ? { ...FALLBACK, ...data } : FALLBACK))
      .catch(() => FALLBACK);
  }
  return cached;
}

/** The arm pair the backend drives (`lelab --robot`); null until known. */
export function useRobotProfile(): RobotProfile | null {
  const { baseUrl } = useApi();
  const [profile, setProfile] = useState<RobotProfile | null>(null);
  useEffect(() => {
    let cancelled = false;
    fetchRobotProfile(baseUrl).then((p) => {
      if (!cancelled) setProfile(p);
    });
    return () => {
      cancelled = true;
    };
  }, [baseUrl]);
  return profile;
}

/**
 * URDF of the follower the backend drives; null until known. Pass `bimanual`
 * for a robot paired with a right arm to get the two-arm model, when there is one.
 */
export function useDefaultUrdfPath(bimanual = false): string | null {
  const profile = useRobotProfile();
  if (!profile) return null;
  return (bimanual && profile.bimanual_urdf_path) || profile.urdf_path;
}
