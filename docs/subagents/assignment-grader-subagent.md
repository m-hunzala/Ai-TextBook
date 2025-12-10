# assignment-grader-subagent

This subagent takes a student repository link, runs tests, verifies ROS topics, and returns detailed rubric scores.

## Input Schema

```json
{
  "type": "object",
  "properties": {
    "student_repo_url": {
      "type": "string",
      "description": "URL to the student's Git repository",
      "pattern": "^https://.*\\.git$"
    },
    "assignment_requirements": {
      "type": "object",
      "description": "Requirements for the specific assignment",
      "properties": {
        "package_name": {
          "type": "string",
          "description": "Expected ROS package name"
        },
        "required_nodes": {
          "type": "array",
          "description": "Names of required ROS nodes",
          "items": {
            "type": "string"
          }
        },
        "required_topics": {
          "type": "array",
          "description": "List of required ROS topics",
          "items": {
            "type": "object",
            "properties": {
              "name": {"type": "string", "description": "Topic name"},
              "type": {"type": "string", "description": "Message type"},
              "direction": {"type": "string", "enum": ["publish", "subscribe", "both"]}
            },
            "required": ["name", "type", "direction"]
          }
        },
        "required_services": {
          "type": "array",
          "description": "List of required ROS services",
          "items": {
            "type": "object",
            "properties": {
              "name": {"type": "string", "description": "Service name"},
              "type": {"type": "string", "description": "Service type"}
            },
            "required": ["name", "type"]
          }
        },
        "required_parameters": {
          "type": "array",
          "description": "List of required ROS parameters",
          "items": {
            "type": "object",
            "properties": {
              "name": {"type": "string", "description": "Parameter name"},
              "type": {"type": "string", "description": "Parameter type"},
              "expected_value": {"type": "string", "description": "Expected parameter value (optional)"}
            },
            "required": ["name", "type"]
          }
        }
      },
      "required": ["package_name", "required_nodes"]
    },
    "grading_rubric": {
      "type": "object",
      "description": "Rubric for grading the assignment",
      "properties": {
        "total_points": {
          "type": "number",
          "description": "Total points for the assignment",
          "minimum": 1
        },
        "criteria": {
          "type": "array",
          "description": "Grading criteria",
          "items": {
            "type": "object",
            "properties": {
              "name": {"type": "string", "description": "Name of the criterion"},
              "description": {"type": "string", "description": "Description of what is being evaluated"},
              "points": {"type": "number", "description": "Points allocated to this criterion"},
              "test_type": {
                "type": "string",
                "description": "Type of test to perform",
                "enum": [
                  "topic_validation",
                  "node_running",
                  "service_available",
                  "parameter_set",
                  "custom_test",
                  "build_success",
                  "unit_tests",
                  "integration_tests"
                ]
              },
              "test_config": {
                "type": "object",
                "description": "Configuration for the test"
              }
            },
            "required": ["name", "description", "points", "test_type"]
          }
        }
      },
      "required": ["total_points", "criteria"]
    },
    "runtime_config": {
      "type": "object",
      "description": "Configuration for running the tests",
      "properties": {
        "timeout": {
          "type": "number",
          "description": "Timeout for tests in seconds",
          "default": 60,
          "minimum": 1
        },
        "ros_version": {
          "type": "string",
          "description": "ROS version to use",
          "enum": ["ros1", "ros2"],
          "default": "ros2"
        },
        "dependencies": {
          "type": "array",
          "description": "Additional dependencies to install",
          "items": {
            "type": "string"
          }
        },
        "launch_file": {
          "type": "string",
          "description": "Launch file to run for testing"
        }
      }
    }
  },
  "required": ["student_repo_url", "assignment_requirements", "grading_rubric"]
}
```

## Output Schema

```json
{
  "type": "object",
  "properties": {
    "student_repo_url": {
      "type": "string",
      "description": "URL of the student's repository that was graded"
    },
    "overall_score": {
      "type": "number",
      "description": "Overall score for the assignment",
      "minimum": 0
    },
    "total_points": {
      "type": "number",
      "description": "Total possible points for the assignment"
    },
    "percentage": {
      "type": "number",
      "description": "Percentage score",
      "minimum": 0,
      "maximum": 100
    },
    "grade": {
      "type": "string",
      "description": "Letter grade (A, B, C, D, F)"
    },
    "detailed_results": {
      "type": "array",
      "description": "Detailed results for each grading criterion",
      "items": {
        "type": "object",
        "properties": {
          "criterion_name": {
            "type": "string",
            "description": "Name of the evaluated criterion"
          },
          "points_earned": {
            "type": "number",
            "description": "Points earned for this criterion"
          },
          "points_possible": {
            "type": "number",
            "description": "Maximum points for this criterion"
          },
          "status": {
            "type": "string",
            "description": "Status of the criterion",
            "enum": ["pass", "fail", "partial", "not_evaluated"]
          },
          "details": {
            "type": "string",
            "description": "Detailed feedback about this criterion"
          },
          "test_output": {
            "type": "string",
            "description": "Output from the test execution"
          }
        },
        "required": ["criterion_name", "points_earned", "points_possible", "status", "details"]
      }
    },
    "build_results": {
      "type": "object",
      "properties": {
        "success": {
          "type": "boolean",
          "description": "Whether the package built successfully"
        },
        "build_output": {
          "type": "string",
          "description": "Output from the build process"
        },
        "errors": {
          "type": "array",
          "description": "List of build errors",
          "items": {
            "type": "string"
          }
        }
      },
      "required": ["success"]
    },
    "runtime_results": {
      "type": "object",
      "properties": {
        "nodes_running": {
          "type": "array",
          "description": "List of nodes that were running",
          "items": {
            "type": "string"
          }
        },
        "topics_detected": {
          "type": "array",
          "description": "List of topics detected during execution",
          "items": {
            "type": "object",
            "properties": {
              "name": {"type": "string"},
              "type": {"type": "string"},
              "publishers": {"type": "number"},
              "subscribers": {"type": "number"}
            }
          }
        },
        "services_detected": {
          "type": "array",
          "description": "List of services detected during execution",
          "items": {
            "type": "object",
            "properties": {
              "name": {"type": "string"},
              "type": {"type": "string"}
            }
          }
        },
        "parameters_detected": {
          "type": "array",
          "description": "List of parameters detected during execution",
          "items": {
            "type": "object",
            "properties": {
              "name": {"type": "string"},
              "type": {"type": "string"},
              "value": {}
            }
          }
        }
      }
    },
    "summary": {
      "type": "string",
      "description": "Summary of the grading results"
    },
    "recommendations": {
      "type": "array",
      "description": "Recommendations for the student to improve",
      "items": {
        "type": "string"
      }
    }
  },
  "required": [
    "student_repo_url",
    "overall_score",
    "total_points",
    "percentage",
    "grade",
    "detailed_results",
    "build_results",
    "runtime_results",
    "summary"
  ]
}
```

## Example Invocation

```json
{
  "student_repo_url": "https://github.com/student/robotics_assignment1.git",
  "assignment_requirements": {
    "package_name": "turtlebot_controller",
    "required_nodes": [
      "velocity_controller",
      "sensor_processor"
    ],
    "required_topics": [
      {
        "name": "/cmd_vel",
        "type": "geometry_msgs/Twist",
        "direction": "publish"
      },
      {
        "name": "/laser_scan",
        "type": "sensor_msgs/LaserScan",
        "direction": "subscribe"
      }
    ],
    "required_services": [
      {
        "name": "/reset_simulation",
        "type": "std_srvs/Empty"
      }
    ],
    "required_parameters": [
      {
        "name": "max_velocity",
        "type": "double",
        "expected_value": "0.5"
      }
    ]
  },
  "grading_rubric": {
    "total_points": 100,
    "criteria": [
      {
        "name": "Build Success",
        "description": "Package builds without errors",
        "points": 20,
        "test_type": "build_success"
      },
      {
        "name": "Required Nodes",
        "description": "All required nodes are present and runnable",
        "points": 30,
        "test_type": "node_running",
        "test_config": {
          "nodes": ["velocity_controller", "sensor_processor"]
        }
      },
      {
        "name": "Topic Communication",
        "description": "Correct topics are published and subscribed",
        "points": 25,
        "test_type": "topic_validation",
        "test_config": {
          "topics": [
            { "name": "/cmd_vel", "type": "geometry_msgs/Twist" },
            { "name": "/laser_scan", "type": "sensor_msgs/LaserScan" }
          ]
        }
      },
      {
        "name": "Unit Tests",
        "description": "All unit tests pass",
        "points": 15,
        "test_type": "unit_tests"
      },
      {
        "name": "Code Quality",
        "description": "Code follows ROS/Python best practices",
        "points": 10,
        "test_type": "custom_test",
        "test_config": {
          "checks": ["pep8", "documentation", "comment_quality"]
        }
      }
    ]
  },
  "runtime_config": {
    "timeout": 60,
    "ros_version": "ros2",
    "dependencies": [
      "geometry_msgs",
      "sensor_msgs",
      "std_srvs"
    ],
    "launch_file": "turtlebot_controller.launch.py"
  }
}
```

## Agent Skill Description

The **assignment-grader-subagent** is an automated grading system for ROS-based assignments that evaluates student repositories against specified requirements. This agent checks for proper package structure, required nodes, topics, services, and parameters while running unit and integration tests. It provides detailed feedback for each grading criterion, including build results, runtime behavior, and code quality assessment. The system implements fair and consistent evaluation by comparing student implementations against a detailed rubric with configurable test parameters. The agent generates comprehensive reports with specific feedback to help students understand their mistakes and improve their implementations, making it an invaluable tool for educators teaching ROS-based robotics courses.