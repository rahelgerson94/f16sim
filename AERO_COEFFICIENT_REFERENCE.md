# F-16 Aero Coefficient Reference

This sheet maps NASA TP-1538 notation to the Matlab repo implementation in:

- [preprocess_F16_AeroData.m](/Users/rahelmizrahi/Documents/Aero/F16-Model-Matlab/preprocess_F16_AeroData.m)
- [F16AeroFM.m](/Users/rahelmizrahi/Documents/Aero/F16-Model-Matlab/F16AeroFM.m)
- [NASA-79-tp1538-2.txt](/Users/rahelmizrahi/Documents/Aero/F16-Model-Matlab/NASA-79-tp1538-2.txt)

## 1. Core notation

- `Cx, Cy, Cz`: body-axis force coefficients
- `Cl, Cm, Cn`: body-axis moment coefficients
- `Cxq, Czq, Cmq`: derivatives with respect to `q cbar / (2V)`
- `Cyp, Clp, Cnp`: derivatives with respect to `p b / (2V)`
- `Cyr, Clr, Cnr`: derivatives with respect to `r b / (2V)`

Important sign/axis note:

- These are body-axis coefficients, not wind-axis `CL/CD`.
- For standard aircraft body axes, positive body `z` points downward, so lift usually corresponds to negative `Cz`.

## 2. NASA notation -> repo variable

| NASA term | Meaning | Repo variable / use | Status |
|---|---|---|---|
| `Cx(alpha,beta,delta_e)` | baseline `X`-force coefficient | `F16AeroData.Cx`, `Cx` | Implemented |
| `Cy(alpha,beta)` | baseline `Y`-force coefficient | `F16AeroData.Cy`, `Cy` | Implemented |
| `Cz(alpha,beta,delta_e)` | baseline `Z`-force coefficient | `F16AeroData.Cz`, `Cz` | Implemented |
| `Cl(alpha,beta,...)` | baseline rolling-moment coefficient | `F16AeroData.Cl`, `Cl` | Implemented |
| `Cm(alpha,beta,delta_e)` | baseline pitching-moment coefficient | `F16AeroData.Cm`, `Cm` | Implemented |
| `Cn(alpha,beta,...)` | baseline yawing-moment coefficient | `F16AeroData.Cn`, `Cn` | Implemented |
| `Delta Cx_lef` etc. | leading-edge flap static increments | `Cx_lef`, `Cy_lef`, `...`, then `delta_*_lef` in code | Implemented |
| `Cxq, Czq, Cmq` | pitch-rate derivative family | `F16AeroData.Cxq`, `Czq`, `Cmq` | Implemented |
| `Cyp, Cyr, Clp, Clr, Cnp, Cnr` | roll/yaw damping family | matching repo names | Implemented |
| `Delta Cxq_lef` etc. | LEF increments to damping derivatives | `deltaCxq_lef`, `deltaCzq_lef`, `...` | Implemented |
| `Delta Cy_da=20`, `Delta Cn_da=20`, `Delta Cl_da=20` | aileron control increments | `Cy_a20`, `Cn_a20`, `Cl_a20` | Implemented |
| `Delta Cy_da=20,lef`, `Delta Cn_da=20,lef`, `Delta Cl_da=20,lef` | aileron + LEF interaction | `Cy_a20_lef`, `Cn_a20_lef`, `Cl_a20_lef` | Implemented |
| `Delta Cy_dr=30`, `Delta Cn_dr=30`, `Delta Cl_dr=30` | rudder control increments | `Cy_r30`, `Cn_r30`, `Cl_r30` | Implemented |
| `Delta Cn_beta(alpha)` | extra nonlinear yaw stability increment | `deltaCnbeta` | Implemented |
| `Delta Cl_beta(alpha)` | extra nonlinear roll stability increment | `deltaClbeta` | Implemented |
| `Delta Cm(alpha)` | extra nonlinear pitch increment | `deltaCm` | Implemented |
| `eta_el(delta_e)` | elevator effectiveness factor | `eta_el` | Implemented |
| `Delta Cx_sb(alpha)` | speed-brake increment to `Cx` | no repo equivalent | Omitted |
| `Delta Cz_sb(alpha)` | speed-brake increment to `Cz` | no repo equivalent | Omitted |
| `Delta Cm_sb(alpha)` | speed-brake increment to `Cm` | no repo equivalent | Omitted |
| `Delta Cm_ds` | deep-stall pitch increment | `delta_Cm_ds = 0` | Omitted |
| Mach-scheduled aero | direct Mach dependence | no repo table dimension | Omitted |
| `alpha_dot`, `beta_dot` aero derivatives | unsteady aero derivatives | no repo terms | Omitted |

## 3. What the "other data" block means

The block beginning near [preprocess_F16_AeroData.m](/Users/rahelmizrahi/Documents/Aero/F16-Model-Matlab/preprocess_F16_AeroData.m#L55) is not miscellaneous noise. It is the finite-deflection and extra-increment content from NASA Appendix B.

| Repo name | Meaning |
|---|---|
| `Cy_r30`, `Cn_r30`, `Cl_r30` | full rudder reference tables at `delta_r = 30 deg` |
| `Cy_a20`, `Cn_a20`, `Cl_a20` | full aileron reference tables at `delta_a = 20 deg` |
| `Cy_a20_lef`, `Cn_a20_lef`, `Cl_a20_lef` | extra aileron effect when LEF is active |
| `deltaCnbeta`, `deltaClbeta` | added nonlinear `beta`-slope increments |
| `deltaCm` | added nonlinear pitch-moment increment |
| `eta_el` | elevator effectiveness schedule |

## 4. Total coefficient build-up in the repo

The repo builds total coefficients in [F16AeroFM.m](/Users/rahelmizrahi/Documents/Aero/F16-Model-Matlab/F16AeroFM.m#L95).

Useful forms:

```text
Cx_tot = baseline Cx + LEF static increment + q-damping contribution
Cz_tot = baseline Cz + LEF static increment + q-damping contribution
Cm_tot = baseline Cm * eta_el + CG correction + LEF increment + q-damping + deltaCm + deltaCm_ds
Cy_tot = baseline Cy + LEF increment + aileron increment + rudder increment + p/r damping
Cn_tot = baseline Cn + LEF increment + CG side-force correction + aileron increment + rudder increment + p/r damping + deltaCnbeta * beta
Cl_tot = baseline Cl + LEF increment + aileron increment + rudder increment + p/r damping + deltaClbeta * beta
```

## 5. How to estimate control derivatives from the available tables

NASA/repo mostly gives finite-deflection tables, not symbolic small-signal derivatives for control surfaces.

Practical approximations:

### Rudder

Use the `r30` tables:

```text
Cy_delta_r(alpha,beta) ~= (Cy_r30 - Cy_baseline) / 30 deg
Cn_delta_r(alpha,beta) ~= (Cn_r30 - Cn_baseline) / 30 deg
Cl_delta_r(alpha,beta) ~= (Cl_r30 - Cl_baseline) / 30 deg
```

If you want derivatives per radian instead of per degree:

```text
Cy_delta_r [per rad] ~= (Cy_r30 - Cy_baseline) / (30 deg * pi/180)
```

### Aileron

Use the `a20` tables:

```text
Cy_delta_a(alpha,beta) ~= (Cy_a20 - Cy_baseline) / 20 deg
Cn_delta_a(alpha,beta) ~= (Cn_a20 - Cn_baseline) / 20 deg
Cl_delta_a(alpha,beta) ~= (Cl_a20 - Cl_baseline) / 20 deg
```

With LEF interaction included:

```text
effective Delta Cy_a ~= Delta Cy_a20 + Delta Cy_a20_lef * dlef
effective Delta Cn_a ~= Delta Cn_a20 + Delta Cn_a20_lef * dlef
effective Delta Cl_a ~= Delta Cl_a20 + Delta Cl_a20_lef * dlef
```

Then divide by `20 deg` or `20 deg * pi/180` depending on desired units.

### Elevator / stabilator

For pitch, the repo uses the full `Cm(alpha,beta,delta_e)` table plus `eta_el(delta_e)`.

A local elevator derivative can be estimated numerically from the table:

```text
Cm_delta_e(alpha,beta,delta_e0)
  ~= [Cm(alpha,beta,delta_e0 + h) - Cm(alpha,beta,delta_e0 - h)] / (2h)
```

with a small `h`, for example `1 deg`.

The same idea works for `Cx_delta_e` and `Cz_delta_e` using the `Cx` and `Cz` tables.

## 6. What you can and cannot get from this model

You can answer well:

- `Cx, Cy, Cz, Cl, Cm, Cn`
- dependence on `alpha`, `beta`
- damping wrt `p, q, r`
- approximate control derivatives wrt aileron, rudder, elevator
- LEF-induced increments

You cannot get directly from this repo/source report:

- true Mach derivatives
- `alpha_dot` aerodynamic derivatives
- `beta_dot` aerodynamic derivatives
- speed-brake aero effects
- deep-stall recovery increment `Delta Cm_ds`
- post-stall asymmetry effects discussed in the report

## 7. Important caveats

- The source model is low-speed and high-alpha focused, not a full-envelope F-16 aerodynamic model.
- Some report conclusions depend on control laws in Appendix A, not just aerodynamic tables.
- One likely implementation issue exists in [F16AeroFM.m](/Users/rahelmizrahi/Documents/Aero/F16-Model-Matlab/F16AeroFM.m#L102):

```text
dZdq = (cbar/(2*Vt)) * (Czq + delta_Cz_lef * dlef)
```

This likely should use `delta_Czq_lef` rather than `delta_Cz_lef`.

## 8. Best next questions

Natural follow-ups from here:

- derive `Cm_delta_e`, `Cz_delta_e`, `Cx_delta_e` numerically from the existing tables
- derive `Cl_delta_a`, `Cn_delta_a`, `Cy_delta_a`, `Cl_delta_r`, `Cn_delta_r`, `Cy_delta_r`
- build `alpha` and `beta` sensitivity maps of `Cx, Cy, Cz, Cl, Cm, Cn`
- add the omitted `*_sb` and `deltaCm_ds` terms to the repo
