use clap::Parser;
use gmt_dos_clients_fem::{solvers::Exponential, DiscreteStateSpace};
use gmt_fem::FEM;

fn frequency_base2_histogram<'a>(nu: &[f64], max_nu: f64) -> Vec<usize> {
    (0..)
        .map_while(|i| {
            let upper = 2i32 << i;
            let lower = if i == 0 { 0 } else { upper >> 1 };
            if lower as f64 > max_nu {
                None
            } else {
                Some(
                    nu.iter()
                        .filter(|&&nu| nu >= lower as f64 && nu < upper as f64)
                        .enumerate()
                        .last()
                        .map_or_else(|| 0, |(i, _)| i + 1),
                )
            }
        })
        .collect()
}

fn model_reduction(
    k: usize,
    nu_hsv: &Vec<(f64, f64)>,
    max_nu: f64,
    n_mode: usize,
    hsv_threshold: f64,
    max_hsv: f64,
    nu_hist: &mut Vec<Vec<usize>>,
    nu_lower_bound: Option<f64>,
) {
    let red_nu_hsv: Vec<&(f64, f64)> = nu_hsv
        .iter()
        .filter(|(nu, _)| *nu <= nu_lower_bound.unwrap_or_default())
        .chain(
            nu_hsv
                .iter()
                .filter(|(nu, _)| *nu > nu_lower_bound.unwrap_or_default())
                .filter(|(_, hsv)| *hsv > max_hsv * hsv_threshold),
        )
        .collect();

    nu_hist.push(frequency_base2_histogram(
        red_nu_hsv
            .iter()
            .map(|(nu, _)| *nu)
            .collect::<Vec<f64>>()
            .as_slice(),
        max_nu,
    ));

    let min_nu = red_nu_hsv
        .iter()
        .map(|(nu, _)| nu)
        .cloned()
        .fold(f64::INFINITY, f64::min);
    let max_nu = red_nu_hsv
        .iter()
        .map(|(nu, _)| nu)
        .cloned()
        .fold(f64::NEG_INFINITY, f64::max);

    println!(
        r#"
{}. reduced model:
 . hankel singular value threshold: {:.3e} ({:e})
 . # of modes: {} ({:.1})%
 . eigen frequencies range: {:.3?}Hz
    "#,
        k + 1,
        max_hsv * hsv_threshold,
        hsv_threshold,
        red_nu_hsv.len(),
        100. * red_nu_hsv.len() as f64 / n_mode as f64,
        (min_nu, max_nu)
    );
}

#[derive(Parser)]
#[command(
    author = "Rod Conan <rconan@gmto.org>",
    version = "0.1.0",
    about = "FEM properties summary with optional model reduction", long_about = None
)]
pub enum SubCommand {
    #[command(name = "gmt-fem")]
    GmtFem(GmtFem),
}

#[derive(Parser, Debug)]
pub struct GmtFem {
    /// Hankel singular value threshold
    hsv: Option<f64>,
    /// Hankel singular value lower relative threshold (log10)
    #[arg(long, allow_hyphen_values = true)]
    hsv_threshold: Option<i32>,
    /// Modal damping coefficient
    #[arg(long)]
    damping: Option<f64>,
    /// Frequency lower bound for Hankel singular value truncation (default: 0Hz)
    #[arg(long)]
    freq: Option<f64>,
    /// Names of inputs to down select to
    #[arg(short, long, use_value_delimiter = true)]
    inputs: Option<Vec<String>>,
    /// Names of outputs to down select to
    #[arg(short, long, use_value_delimiter = true)]
    outputs: Option<Vec<String>>,
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let subc = SubCommand::parse();
    let SubCommand::GmtFem(cli) = subc;

    let fem = FEM::from_env()?;

    let nu = fem.eigen_frequencies.clone();
    let max_nu = nu.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
    let mut nu_hist = vec![frequency_base2_histogram(&nu, max_nu)];

    let mut state_space = if let Some(zeta) = cli.damping {
        DiscreteStateSpace::<Exponential>::from(fem).proportional_damping(zeta)
    } else {
        DiscreteStateSpace::<Exponential>::from(fem)
    };
    state_space.fem_info();
    let (_state_space, hsv) = match (cli.inputs, cli.outputs) {
        (None, None) => {
            let hsv = state_space.hankel_singular_values()?;
            (state_space, hsv)
        }
        (None, Some(outputs)) => {
            state_space = state_space.outs_by_name(outputs)?;
            let hsv = state_space.reduced_hankel_singular_values()?;
            (state_space, hsv)
        }
        (Some(inputs), None) => {
            state_space = state_space.ins_by_name(inputs)?;
            let hsv = state_space.reduced_hankel_singular_values()?;
            (state_space, hsv)
        }
        (Some(inputs), Some(outputs)) => {
            state_space = state_space.ins_by_name(inputs)?.outs_by_name(outputs)?;
            let hsv = state_space.reduced_hankel_singular_values()?;
            (state_space, hsv)
        }
    };
    let max_hsv = hsv.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
    let nu_hsv: Vec<_> = nu.iter().cloned().zip(hsv.into_iter()).collect();

    let n_mode = nu_hsv.len();

    print!(
        r#"
HANKEL SINGULAR VALUES MODEL REDUCTION
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
"#
    );

    let model_log_reduction: Vec<i32> = (0..4)
        .map(|i| cli.hsv_threshold.unwrap_or(-6i32) + i)
        .collect();

    for (k, exp) in model_log_reduction.into_iter().enumerate() {
        let hsv_threshold = 10f64.powi(exp);
        model_reduction(
            k,
            &nu_hsv,
            max_nu,
            n_mode,
            hsv_threshold,
            max_hsv,
            &mut nu_hist,
            cli.freq,
        );
    }

    if let Some(hsv_threshold) = cli.hsv {
        model_reduction(
            5,
            &nu_hsv,
            max_nu,
            n_mode,
            hsv_threshold,
            max_hsv,
            &mut nu_hist,
            cli.freq,
        );
    }

    println!(" {}", "-".repeat(43));
    println!(" |{:^41}|", "Models Frequency Histograms");
    println!(" |{}|", "-".repeat(41));
    println!(" |{:^6}|{:^34}|", "Bin", "Models");
    print!(" |{:^5}", "Hz");
    for i in 0..nu_hist.len() {
        print!(" | {:^4}", i);
    }
    println!(" |");
    println!(" {}|", "|------".repeat(1 + nu_hist.len()));
    let n_bin = nu_hist[0].len();
    for i in 0..n_bin {
        let upper = 2 << i;
        print!(" |{:>5} ", upper);
        for hist in &nu_hist {
            print!("|{:>5} ", hist[i]);
        }
        println!("|");
    }
    println!(" {}", "-".repeat(43));

    Ok(())
}

/*  -------------------------------------------
|       Models Frequency Histograms       |
|-----------------------------------------|
| Bin  |              Models              |
| Hz   |  0   |  1   |  2   |  3   |  4   |
|------|------|------|------|------|------|
|    2 |    3 |    1 |    0 |    0 |    0 |
|    4 |    4 |    1 |    1 |    0 |    0 |
|    8 |   18 |   13 |    9 |    4 |    1 |
|   16 |   95 |   84 |   76 |   56 |   19 |
|   32 |  509 |  502 |  490 |  446 |  382 |
|   64 | 2090 | 2084 | 2032 | 1740 | 1068 |
|  128 | 6583 | 6501 | 6025 | 4450 | 1817 |
|  256 |  455 |  449 |  439 |  413 |  341 |
|  512 |  866 |  865 |  858 |  845 |  749 |
| 1024 |   28 |   28 |   28 |   28 |   23 |
-------------------------------------------
-------------------------------------------
 |       Models Frequency Histograms       |
 |-----------------------------------------|
 | Bin  |              Models              |
 | Hz   |  0   |  1   |  2   |  3   |  4   |
 |------|------|------|------|------|------|
 |    2 |    3 |    3 |    3 |    3 |    3 |
 |    4 |    4 |    4 |    4 |    4 |    4 |
 |    8 |   18 |   18 |   18 |   18 |   18 |
 |   16 |   95 |   95 |   95 |   95 |   95 |
 |   32 |  509 |  509 |  509 |  509 |  509 |
 |   64 | 2090 | 2001 | 1740 | 1328 | 1138 |
 |  128 | 6583 | 4450 | 1817 |  295 |   22 |
 |  256 |  455 |  413 |  341 |  209 |  128 |
 |  512 |  866 |  845 |  749 |  520 |  289 |
 | 1024 |   28 |   28 |   23 |    6 |    3 |
 -------------------------------------------*/
