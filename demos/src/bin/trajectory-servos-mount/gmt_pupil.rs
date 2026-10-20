//! GMT pupil image for the flowchart block of the optical model
//!
//! The 7 segments of the GMT pupil, as seen from the sky, with the segments of the assembly highlighted.
//! The assembly is set at build time with the environment variable `ASSEMBLY`
//! (e.g. `ASSEMBLY=1,2,7`, all the segments if it is not set, see `gmt_dos_clients_io::assembly`).

use gmt_dos_clients_io::assembly::Assembly;

/// The GMT assembly
struct Gmt;
impl Assembly for Gmt {}

/// Distance of the outer segment centers from the optical axis [m]
const L: f64 = 8.710;
/// Diameter of the clear aperture of a segment [m]
const D: f64 = 8.365;
/// Diameter of the hole of the center segment [m]
const HOLE: f64 = 2.4412;

/// Returns the segment ids of the assembly
pub fn assembly() -> Vec<u8> {
    <Gmt as Assembly>::SIDS.to_vec()
}

/// Returns the SVG image of the GMT pupil with the segments of the assembly highlighted
pub fn svg() -> String {
    svg_of(&assembly())
}

/// Returns the SVG image of the GMT pupil with the segments `active` highlighted
fn svg_of(active: &[u8]) -> String {
    let r = D / 2.;
    let segments: String = (1..=7u8)
        .map(|sid| {
            // the outer segments are at angles pi(3-2k)/6, k=0,...,5, from the x-axis,
            // segment #1 on the y-axis, and segment #7 at the center (CEO convention);
            // the SVG y-axis points down
            let (x, y) = if sid == 7 {
                (0., 0.)
            } else {
                let o = std::f64::consts::PI * (3. - 2. * (sid - 1) as f64) / 6.;
                (L * o.cos(), -L * o.sin())
            };
            let class = if active.contains(&sid) { "on" } else { "off" };
            // a disk, or an annulus for the center segment
            let shape = if sid == 7 {
                let h = HOLE / 2.;
                format!(
                    r#"<path class="{class}" fill-rule="evenodd" d="M{} {y} a{r} {r} 0 1 0 {} 0 a{r} {r} 0 1 0 {} 0 Z M{} {y} a{h} {h} 0 1 0 {} 0 a{h} {h} 0 1 0 {} 0 Z"/>"#,
                    x - r,
                    2. * r,
                    -2. * r,
                    x - h,
                    2. * h,
                    -2. * h
                )
            } else {
                format!(r#"<circle class="{class}" cx="{x:.3}" cy="{y:.3}" r="{r}"/>"#)
            };
            let label_y = if sid == 7 { y + r * 0.62 } else { y };
            format!(
                r#"{shape}<text class="{class}" x="{x:.3}" y="{label_y:.3}">S{sid}</text>"#
            )
        })
        .collect();
    format!(
        r##"<svg xmlns="http://www.w3.org/2000/svg" viewBox="-13 -13 26 26" width="260" height="260">
<style>
circle, path {{ stroke-width: 0.25; }}
circle.on, path.on {{ fill: #5b7bb5; stroke: #2f4f8a; }}
circle.off, path.off {{ fill: #9aa0a6; fill-opacity: 0.18; stroke: #9aa0a6; stroke-dasharray: 0.6 0.4; }}
text {{ font: bold 2.2px system-ui, sans-serif; text-anchor: middle; dominant-baseline: central; stroke: none; }}
text.on {{ fill: #ffffff; }}
text.off {{ fill: #9aa0a6; }}
</style>
{segments}
</svg>
"##
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn segments() {
        let svg = svg();
        assert_eq!(svg.matches("<circle").count(), 6);
        assert_eq!(svg.matches("<path").count(), 1);
        assert_eq!(svg.matches(r#"class="on""#).count(), 2 * assembly().len());
        // segment #1 on the y-axis, above the center
        assert!(svg.contains(r#"cx="0.000" cy="-8.710""#));
        let svg = svg_of(&[1, 2, 7]);
        assert_eq!(svg.matches(r#"class="on""#).count(), 6);
        assert_eq!(svg.matches(r#"class="off""#).count(), 8);
    }

    #[test]
    #[ignore = "writes the pupil images to GMT_PUPIL_OUT"]
    fn images() {
        let out = std::path::PathBuf::from(std::env::var("GMT_PUPIL_OUT").unwrap());
        std::fs::write(out.join("pupil_all.svg"), svg_of(&[1, 2, 3, 4, 5, 6, 7])).unwrap();
        std::fs::write(out.join("pupil_127.svg"), svg_of(&[1, 2, 7])).unwrap();
    }
}
