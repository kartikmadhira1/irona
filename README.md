# Irona
[![Build Status](https://travis-ci.org/kartikmadhira1/irona.svg?branch=master)](https://travis-ci.org/kartikmadhira1/irona)
[![Coverage Status](https://coveralls.io/repos/github/kartikmadhira1/irona/badge.svg?branch=master)](https://coveralls.io/github/kartikmadhira1/irona?branch=master)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
---

## Overview
With the advent of technology the emergence of robotics industry is quite evident. As the industry is
evolving efforts are made to deploy robots in situations where the interaction of heavy machinery and
humans is redundant. One such application is deployment in warehouse to move packages from one point
to another. Warehouse setup poses several challenges to achieve this task. One of the challenges is to
identify and locate the desired package in the cluttered environment.
We propose to design a collection robot for fulfilling orders at the ACME warehouse. We will make use
of a Tiago++ ground robot to scan the warehouse, identify orders to be picked up, collect them and
place at the final checkout. Tiago++ is a mobile manipulator with two arms along with several other
components to assist picking and grasping of several shaped objects. The proposed package takes the
map of the environment and the ArUco marker’s information associated with the package as the input
and ultimately collect the box and deliver it at the desired location (checkout).
