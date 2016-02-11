/*********************************************************************
 Autonomous Motion Department
 Max-Planck Intelligent Systems
 Prof. Stefan Schaal
 *********************************************************************
 \remarks      ...

 \file         EigenQpSolver.h

 \author       Alexander Herzog
 \date         Aug 1, 2013

 *********************************************************************/

#ifndef EIGENQPSOLVER_H_
#define EIGENQPSOLVER_H_

#include <stdio.h>
#include <iostream>
#include <Eigen/Dense>

namespace floating_base_utilities
{

class EigenQpSolver
{
public:
  EigenQpSolver();
  virtual ~EigenQpSolver();

  static double solve_quadprog(Eigen::MatrixXd & Hessian,  Eigen::VectorXd & g0,
                               const Eigen::MatrixXd & CE, const Eigen::VectorXd & ce0,
                               const Eigen::MatrixXd & CI, const Eigen::VectorXd & ci0,
                               Eigen::VectorXd& x);

  /**
   * Hess^-1 = Hessian_factor_inv * Hessian_factor_inv.transpose()
   */
  template<typename invMatDer>
  static double solve_quadprog(Eigen::MatrixXd & Hessian,  Eigen::VectorXd & g0,
                               const Eigen::MatrixXd & CE, const Eigen::VectorXd & ce0,
                               const Eigen::MatrixXd & CI, const Eigen::VectorXd & ci0,
                               Eigen::VectorXd& x, Eigen::MatrixBase<invMatDer>& Hessian_factor_inv);

private:
  template<typename Scalar>
  static inline Scalar distance(Scalar a, Scalar b);
  static inline void compute_d(Eigen::VectorXd &d, const Eigen::MatrixXd& Hessian_factor_inv, const Eigen::VectorXd& np)
  {
    d = Hessian_factor_inv.adjoint() * np;
  }

  template<typename invMatDer>
  static inline void update_z(Eigen::VectorXd& z, const Eigen::MatrixBase<invMatDer>& Hessian_factor_inv, const Eigen::VectorXd& d,  int iq)
  {
    z = Hessian_factor_inv.rightCols(z.size()-iq) * d.tail(d.size()-iq);
  }

  static inline void update_r(const Eigen::MatrixXd& R, Eigen::VectorXd& r, const Eigen::VectorXd& d, int iq)
  {
    r.head(iq)= R.topLeftCorner(iq,iq).triangularView<Eigen::Upper>().solve(d.head(iq));
  }

  template<typename invMatDer>
  static bool add_constraint(Eigen::MatrixXd& R, Eigen::MatrixBase<invMatDer>& Hessian_factor_inv, Eigen::VectorXd& d, int& iq, double& R_norm);
  template<typename invMatDer>
  static void delete_constraint(Eigen::MatrixXd& R, Eigen::MatrixBase<invMatDer>& Hessian_factor_inv, Eigen::VectorXi& A, Eigen::VectorXd& u,  int p, int& iq, int l);

};

template<typename Scalar>
inline Scalar EigenQpSolver::distance(Scalar a, Scalar b)
{
  Scalar a1, b1, t;
  a1 = std::abs(a);
  b1 = std::abs(b);
  if (a1 > b1)
  {
    t = (b1 / a1);
    return a1 * std::sqrt(1.0 + t * t);
  }
  else
    if (b1 > a1)
    {
      t = (a1 / b1);
      return b1 * std::sqrt(1.0 + t * t);
    }
  return a1 * std::sqrt(2.0);
}

template<typename invMatDer>
double EigenQpSolver::solve_quadprog(Eigen::MatrixXd & Hessian,  Eigen::VectorXd & g0,
                             const Eigen::MatrixXd & CE, const Eigen::VectorXd & ce0,
                             const Eigen::MatrixXd & CI, const Eigen::VectorXd & ci0,
                             Eigen::VectorXd& x, Eigen::MatrixBase<invMatDer>& Hessian_factor_inv)
{
  int i, j, k, l; /* indices */
  int ip, me, mi;
  int n=g0.size();  int p=ce0.size();  int m=ci0.size();
  Eigen::MatrixXd R(Hessian.rows(),Hessian.cols());

  Eigen::VectorXd s(m+p), z(n), r(m + p), d(n),  np(n), u(m + p);
  Eigen::VectorXd x_old(n), u_old(m + p);
  double f_value, psi, c1, c2, sum, ss, R_norm;
  const double inf = std::numeric_limits<double>::infinity();
  double t, t1, t2; /* t is the step length, which is the minimum of the partial step length t1
   * and the full step length t2 */
  Eigen::VectorXi A(m + p), A_old(m + p), iai(m + p);
  int q;
  int iq, iter = 0;
  bool iaexcl[m + p];

  me = p; /* number of equality constraints */
  mi = m; /* number of inequality constraints */
  q = 0;  /* size of the active set A (containing the indices of the active constraints) */

  /*
   * Preprocessing phase
   */

  /* compute the trace of the original matrix Hessian */
    c1 = Hessian.trace();



  /* initialize the matrix R */
  d.setZero();
  R.setZero();
  R_norm = 1.0; /* this variable will hold the norm of the matrix R */

  /* compute the inverse of the factorized matrix Hessian^-1, this is the initial value for H */
  // Hessian_factor_inv = L^-T
#ifndef RTEIG_NO_ASSERTS
//    std::cout << "hess*inv norm: " << (Hessian*Hessian_factor_inv*Hessian_factor_inv.transpose() - Eigen::MatrixXd::Identity(Hessian.rows(), Hessian.cols())) << std::endl;
    assert((Hessian*Hessian_factor_inv*Hessian_factor_inv.transpose() - Eigen::MatrixXd::Identity(Hessian.rows(), Hessian.cols())).norm() < 0.0001 && "inverse is weird");
#endif

  c2 = Hessian_factor_inv.trace();
#ifdef TRACE_SOLVER
  print_matrix("Hessian_factor_inv", Hessian_factor_inv, n);
#endif

  /* c1 * c2 is an estimate for cond(Hessian) */

  /*
   * Find the unconstrained minimizer of the quadratic form 0.5 * x Hessian x + g0 x
   * this is a feasible point in the dual space
   * x = Hessian^-1 * g0
   */
  x = Hessian_factor_inv*Hessian_factor_inv.transpose()*g0;

  x = -x;
  /* and compute the current solution value */
  f_value = 0.5 * g0.dot(x);
#ifdef TRACE_SOLVER
  std::cerr << "Unconstrained solution: " << f_value << std::endl;
  print_vector("x", x, n);
#endif

  /* Add equality constraints to the working set A */
  iq = 0;
  for (i = 0; i < me; i++)
  {
    np = CE.col(i);
    compute_d(d, Hessian_factor_inv, np);
    update_z(z, Hessian_factor_inv, d,  iq);
    update_r(R, r, d,  iq);
#ifdef TRACE_SOLVER
    print_matrix("R", R, iq);
    print_vector("z", z, n);
    print_vector("r", r, iq);
    print_vector("d", d, n);
#endif

    /* compute full step length t2: i.e., the minimum step in primal space s.t. the contraint
      becomes feasible */
    t2 = 0.0;
    if (std::abs(z.dot(z)) > std::numeric_limits<double>::epsilon()) // i.e. z != 0
      t2 = (-np.dot(x) - ce0(i)) / z.dot(np);

    x += t2 * z;

    /* set u = u+ */
    u(iq) = t2;
    u.head(iq) -= t2 * r.head(iq);

    /* compute the new solution value */
    f_value += 0.5 * (t2 * t2) * z.dot(np);
    A(i) = -i - 1;

    if (!add_constraint(R, Hessian_factor_inv, d, iq, R_norm))
    {
      // FIXME: it should raise an error
      // Equality constraints are linearly dependent
      printf("quadprog ERROR1\n");
      return f_value;
    }
  }

  /* set iai = K \ A */
  for (i = 0; i < mi; i++)
    iai(i) = i;

  l1:   iter++;
#ifdef TRACE_SOLVER
  print_vector("x", x, n);
#endif
  /* step 1: choose a violated constraint */
  for (i = me; i < iq; i++)
  {
    ip = A(i);
    iai(ip) = -1;
  }

  /* compute s(x) = ci^T * x + ci0 for all elements of K \ A */
  ss = 0.0;
  psi = 0.0; /* this value will contain the sum of all infeasibilities */
  ip = 0; /* ip will be the index of the chosen violated constraint */
  for (i = 0; i < mi; i++)
  {
    iaexcl[i] = true;
    sum = CI.col(i).dot(x) + ci0(i);
    s(i) = sum;
    psi += std::min(0.0, sum);
  }
#ifdef TRACE_SOLVER
  print_vector("s", s, mi);
#endif


  if (std::abs(psi) <= mi * std::numeric_limits<double>::epsilon() * c1 * c2* 100.0)
  {
    /* numerically there are not infeasibilities anymore */
    q = iq;
    return f_value;
  }

  /* save old values for u, x and A */
  u_old.head(iq) = u.head(iq);
  A_old.head(iq) = A.head(iq);
  x_old = x;

  l2: /* Step 2: check for feasibility and determine a new S-pair */
  for (i = 0; i < mi; i++)
  {
    if (s(i) < ss && iai(i) != -1 && iaexcl[i])
    {
      ss = s(i);
      ip = i;
    }
  }
  if (ss >= 0.0)
  {
    q = iq;
    return f_value;
  }

  /* set np = n(ip) */
  np = CI.col(ip);
  /* set u = (u 0)^T */
  u(iq) = 0.0;
  /* add ip to the active set A */
  A(iq) = ip;

#ifdef TRACE_SOLVER
  std::cerr << "Trying with constraint " << ip << std::endl;
  print_vector("np", np, n);
#endif

  l2a:/* Step 2a: determine step direction */
  /* compute z = H np: the step direction in the primal space (through Hessian_factor_inv, see the paper) */
  compute_d(d, Hessian_factor_inv, np);
  update_z(z, Hessian_factor_inv, d, iq);
  /* compute N* np (if q > 0): the negative of the step direction in the dual space */
  update_r(R, r, d, iq);
#ifdef TRACE_SOLVER
  std::cerr << "Step direction z" << std::endl;
  print_vector("z", z, n);
  print_vector("r", r, iq + 1);
  print_vector("u", u, iq + 1);
  print_vector("d", d, n);
  print_ivector("A", A, iq + 1);
#endif

  /* Step 2b: compute step length */
  l = 0;
  /* Compute t1: partial step length (maximum step in dual space without violating dual feasibility */
  t1 = inf; /* +inf */
  /* find the index l s.t. it reaches the minimum of u+(x) / r */
  for (k = me; k < iq; k++)
  {
    double tmp;
    if (r(k) > 0.0 && ((tmp = u(k) / r(k)) < t1) )
    {
      t1 = tmp;
      l = A(k);
    }
  }
  /* Compute t2: full step length (minimum step in primal space such that the constraint ip becomes feasible */
  if (std::abs(z.dot(z))  > std::numeric_limits<double>::epsilon()) // i.e. z != 0
    t2 = -s(ip) / z.dot(np);
  else
    t2 = inf; /* +inf */

  /* the step is chosen as the minimum of t1 and t2 */
  t = std::min(t1, t2);
#ifdef TRACE_SOLVER
  std::cerr << "Step sizes: " << t << " (t1 = " << t1 << ", t2 = " << t2 << ") ";
#endif

  /* Step 2c: determine new S-pair and take step: */

  /* case (i): no step in primal or dual space */
  if (t >= inf)
  {
    /* QPP is infeasible */
    // FIXME: unbounded to raise
    printf("no step in primal or dual space\n");
    q = iq;
    return inf;
  }
  /* case (ii): step in dual space */
  if (t2 >= inf)
  {
    /* set u = u +  t * [-r 1) and drop constraint l from the active set A */
    u.head(iq) -= t * r.head(iq);
    u(iq) += t;
    iai(l) = l;
    delete_constraint(R, Hessian_factor_inv, A, u, p, iq, l);
#ifdef TRACE_SOLVER
    std::cerr << " in dual space: "
        << f_value << std::endl;
    print_vector("x", x, n);
    print_vector("z", z, n);
    print_ivector("A", A, iq + 1);
#endif
    goto l2a;
  }

  /* case (iii): step in primal and dual space */

  x += t * z;
  /* update the solution value */
  f_value += t * z.dot(np) * (0.5 * t + u(iq));

  u.head(iq) -= t * r.head(iq);
  u(iq) += t;
#ifdef TRACE_SOLVER
  std::cerr << " in both spaces: "
      << f_value << std::endl;
  print_vector("x", x, n);
  print_vector("u", u, iq + 1);
  print_vector("r", r, iq + 1);
  print_ivector("A", A, iq + 1);
#endif

  if (t == t2)
  {
#ifdef TRACE_SOLVER
    std::cerr << "Full step has taken " << t << std::endl;
    print_vector("x", x, n);
#endif
    /* full step has taken */
    /* add constraint ip to the active set*/
    if (!add_constraint(R, Hessian_factor_inv, d, iq, R_norm))
    {
      iaexcl[ip] = false;
      delete_constraint(R, Hessian_factor_inv, A, u, p, iq, ip);
#ifdef TRACE_SOLVER
      print_matrix("R", R, n);
      print_ivector("A", A, iq);
#endif
      for (i = 0; i < m; i++)
        iai(i) = i;
      for (i = 0; i < iq; i++)
      {
        A(i) = A_old(i);
        iai(A(i)) = -1;
        u(i) = u_old(i);
      }
      x = x_old;
      goto l2; /* go to step 2 */
    }
    else
      iai(ip) = -1;
#ifdef TRACE_SOLVER
    print_matrix("R", R, n);
    print_ivector("A", A, iq);
#endif
    goto l1;
  }

  /* a patial step has taken */
#ifdef TRACE_SOLVER
  std::cerr << "Partial step has taken " << t << std::endl;
  print_vector("x", x, n);
#endif
  /* drop constraint l */
  iai(l) = l;
  delete_constraint(R, Hessian_factor_inv, A, u, p, iq, l);
#ifdef TRACE_SOLVER
  print_matrix("R", R, n);
  print_ivector("A", A, iq);
#endif

  s(ip) = CI.col(ip).dot(x) + ci0(ip);

#ifdef TRACE_SOLVER
  print_vector("s", s, mi);
#endif
  goto l2a;
}

template<typename invMatDer>
bool EigenQpSolver::add_constraint(Eigen::MatrixXd& R, Eigen::MatrixBase<invMatDer>& Hessian_factor_inv, Eigen::VectorXd& d, int& iq, double& R_norm)
{
  int n=Hessian_factor_inv.rows();
#ifdef TRACE_SOLVER
  std::cerr << "Add constraint " << iq << '/';
#endif
  int i, j, k;
  double cc, ss, h, t1, t2, xny;

  /* we have to find the Givens rotation which will reduce the element
                d(j) to zero.
                if it is already zero we don't have to do anything, except of
                decreasing j */
  for (j = n - 1; j >= iq + 1; j--)
  {
    /* The Givens rotation is done with the matrix (cc cs, cs -cc).
                         If cc is one, then element (j) of d is zero compared with element
                         (j - 1). Hence we don't have to do anything.
                         If cc is zero, then we just have to switch column (j) and column (j - 1)
                         of Hessian_factor_inv. Since we only switch columns in Hessian_factor_inv, we have to be careful how we
                         update d depending on the sign of gs.
                         Otherwise we have to apply the Givens rotation to these columns.
                         The i - 1 element of d has to be updated to h. */
    cc = d(j - 1);
    ss = d(j);
    h = distance(cc, ss);
    if (h == 0.0)
      continue;
    d(j) = 0.0;
    ss = ss / h;
    cc = cc / h;
    if (cc < 0.0)
    {
      cc = -cc;
      ss = -ss;
      d(j - 1) = -h;
    }
    else
      d(j - 1) = h;
    xny = ss / (1.0 + cc);
    for (k = 0; k < n; k++)
    {
      t1 = Hessian_factor_inv(k,j - 1);
      t2 = Hessian_factor_inv(k,j);
      Hessian_factor_inv(k,j - 1) = t1 * cc + t2 * ss;
      Hessian_factor_inv(k,j) = xny * (t1 + Hessian_factor_inv(k,j - 1)) - t2;
    }
  }
  /* update the number of constraints added*/
  iq++;
  /* To update R we have to put the iq components of the d vector
    into column iq - 1 of R
   */
  R.col(iq-1).head(iq) = d.head(iq);
#ifdef TRACE_SOLVER
  std::cerr << iq << std::endl;
#endif

  if (std::abs(d(iq - 1)) <= std::numeric_limits<double>::epsilon() * R_norm)
    // problem degenerate
    return false;
  R_norm = std::max<double>(R_norm, std::abs(d(iq - 1)));
  return true;
}

template<typename invMatDer>
void EigenQpSolver::delete_constraint(Eigen::MatrixXd& R, Eigen::MatrixBase<invMatDer>& Hessian_factor_inv, Eigen::VectorXi& A, Eigen::VectorXd& u,  int p, int& iq, int l)
{

  int n = R.rows();
#ifdef TRACE_SOLVER
  std::cerr << "Delete constraint " << l << ' ' << iq;
#endif
  int i, j, k, qq;
  qq=0;
  double cc, ss, h, xny, t1, t2;

  /* Find the index qq for active constraint l to be removed */
  for (i = p; i < iq; i++)
    if (A(i) == l)
    {
      qq = i;
      break;
    }

  /* remove the constraint from the active set and the duals */
  for (i = qq; i < iq - 1; i++)
  {
    A(i) = A(i + 1);
    u(i) = u(i + 1);
    R.col(i) = R.col(i+1);
  }

  A(iq - 1) = A(iq);
  u(iq - 1) = u(iq);
  A(iq) = 0;
  u(iq) = 0.0;
  for (j = 0; j < iq; j++)
    R(j,iq - 1) = 0.0;
  /* constraint has been fully removed */
  iq--;
#ifdef TRACE_SOLVER
  std::cerr << '/' << iq << std::endl;
#endif

  if (iq == 0)
    return;

  for (j = qq; j < iq; j++)
  {
    cc = R(j,j);
    ss = R(j + 1,j);
    h = distance(cc, ss);
    if (h == 0.0)
      continue;
    cc = cc / h;
    ss = ss / h;
    R(j + 1,j) = 0.0;
    if (cc < 0.0)
    {
      R(j,j) = -h;
      cc = -cc;
      ss = -ss;
    }
    else
      R(j,j) = h;

    xny = ss / (1.0 + cc);
    for (k = j + 1; k < iq; k++)
    {
      t1 = R(j,k);
      t2 = R(j + 1,k);
      R(j,k) = t1 * cc + t2 * ss;
      R(j + 1,k) = xny * (t1 + R(j,k)) - t2;
    }
    for (k = 0; k < n; k++)
    {
      t1 = Hessian_factor_inv(k,j);
      t2 = Hessian_factor_inv(k,j + 1);
      Hessian_factor_inv(k,j) = t1 * cc + t2 * ss;
      Hessian_factor_inv(k,j + 1) = xny * (Hessian_factor_inv(k,j) + t1) - t2;
    }
  }
}
} /* namespace floating_base_utilities */
#endif /* EIGENQPSOLVER_H_ */
