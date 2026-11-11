//Assignment 3
#include "optimization/solvers.h"

#include <Eigen/LU>
#include <Eigen/QR>

#include <cmath>
#include <numbers>


namespace ais4104::optimization {

//TASK: T.1a
//REFERENCE: Algorithm in section 6.2.2 page 228, MR pre-print 2019
praxis::expected<Eigen::VectorXd, praxis::refusal> newton_raphson(const praxis::optimization::problem &p, const Eigen::VectorXd &x0, std::uint32_t max_iterations,
                                                                  std::vector<Eigen::VectorXd> &iterates)
{
    //append x0 to iterates
    //for each iteration:
    //    evaluate the error e(x) with p.error
    //    return x when p.converged holds for e(x)
    //    make the step
    //    append the new x to iterates
    //past max_iterations: return praxis::unexpected(praxis::refusal::no_solution)

    iterates.push_back(x0);

    Eigen::VectorXd xk = x0;
    for (std::uint32_t k = 0; k < max_iterations; ++k)
    {
        auto e = p.error(xk);
        if (!e)
        {
            return praxis::unexpected(praxis::refusal(e.error()));
        }

        auto J = p.jacobian(xk);
        if (!J)
        {
            return praxis::unexpected(praxis::refusal(J.error()));
        }

        if (p.converged(e.value()))
        {
            return xk;
        }

        // TODO: Fix this to use allowed functions
        auto Je = J.value().completeOrthogonalDecomposition().pseudoInverse()*e.value();
        xk += Je;

        iterates.push_back(xk);
    }

    return praxis::unexpected(praxis::refusal::no_solution);
}

//TASK: T.1b
//REFERENCE: thesis excerpt (Eq. 2.59)
praxis::expected<Eigen::VectorXd, praxis::refusal> gradient_descent(const praxis::optimization::problem &p, const Eigen::VectorXd &x0, double step_length,
                                                                    std::uint32_t max_iterations, std::vector<Eigen::VectorXd> &iterates)
{
    //append x0 to iterates
    //for each iteration:
    //    evaluate the gradient of f at x with p.gradient
    //    return x when p.stationary holds for the gradient
    //    make the step
    //    append the new x to iterates
    //past max_iterations: return praxis::unexpected(praxis::refusal::no_solution)
    iterates.push_back(x0);

    Eigen::VectorXd xk = x0;
    for (std::uint32_t k = 0; k < max_iterations; ++k)
    {
        auto g = p.gradient(xk);
        if (!g)
        {
            return praxis::unexpected(praxis::refusal(g.error()));
        }

        if (p.stationary(g.value()))
        {
            return xk;
        }

        xk -= step_length*g.value();
        iterates.push_back(xk);
    }

    return praxis::unexpected(praxis::refusal::no_solution);
}

}
