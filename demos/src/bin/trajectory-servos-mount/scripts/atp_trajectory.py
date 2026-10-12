import sys
import numpy as np
import astropy.units as u
from astropy.time import Time
from astropy.coordinates import EarthLocation, AltAz, SkyCoord, TETE

out = sys.argv[1]
# ATP default pointing (atp.yaml)
loc = EarthLocation(lat=-29.049*u.deg, lon=-70.682*u.deg, height=2514*u.m)
t0 = Time("2018-01-01T04:00:00", scale="utc")
alt0, az0, rot0 = 45.0, 0.0, 0.0
# fix the sky target at the initial alt/az (no refraction)
frame0 = AltAz(obstime=t0, location=loc)
target = SkyCoord(alt=alt0*u.deg, az=az0*u.deg, frame=frame0).icrs

ts = np.arange(0, 301)  # 5 min at 1 Hz
times = t0 + ts*u.s
aa = target.transform_to(AltAz(obstime=times, location=loc))
el = aa.alt.deg
az = np.rad2deg(np.unwrap(aa.az.rad))
if az[0] > 180: az -= 360

# parallactic angle q = atan2(sin H, tan(phi) cos(dec) - sin(dec) cos(H))
lst = times.sidereal_time("apparent", longitude=loc.lon)
tete = target.transform_to(TETE(obstime=times, location=loc))  # equinox-based apparent place, matches apparent LST
ha = (lst - tete.ra).wrap_at(180*u.deg).rad
dec = tete.dec.rad
phi = loc.lat.rad
q = np.unwrap(np.arctan2(np.sin(ha), np.tan(phi)*np.cos(dec) - np.sin(dec)*np.cos(ha)))
rot = rot0 + np.rad2deg(q - q[0])

np.savetxt(out, np.column_stack([ts, az, el, rot]), delimiter=",",
           header="time [s],azimuth [deg],elevation [deg],rotator [deg]", comments="",
           fmt=["%d", "%.9f", "%.9f", "%.9f"])
print(f"target ICRS: ra={target.ra.deg:.6f} dec={target.dec.deg:.6f}")
print(f"az  {az[0]:.6f} -> {az[-1]:.6f} deg (delta {(az[-1]-az[0])*3600:.1f} arcsec)")
print(f"el  {el[0]:.6f} -> {el[-1]:.6f} deg (delta {(el[-1]-el[0])*3600:.1f} arcsec)")
print(f"rot {rot[0]:.6f} -> {rot[-1]:.6f} deg (delta {(rot[-1]-rot[0])*3600:.1f} arcsec); q0={np.rad2deg(q[0]):.4f} deg")
