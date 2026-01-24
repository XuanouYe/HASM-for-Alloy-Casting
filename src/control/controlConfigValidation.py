from typing import Dict, Any, Optional, Union, List
from enum import Enum

class InfillPattern(Enum):
    """
    Infill pattern enumeration.
    """
    GRID = "grid"
    HONEYCOMB = "honeycomb"
    GYROID = "gyroid"
    ADAPTIVE = "adaptive"

class CastingSpeed(Enum):
    """
    Casting speed levels.
    """
    SLOW = "slow"
    MEDIUM = "medium"
    FAST = "fast"

class CuttingStrategy(Enum):
    """
    Cutting strategies.
    """
    CONTOUR = "contour"
    POCKET = "pocket"
    HYBRID = "hybrid"

validationRules: Dict[str, Dict[str, Dict[str, Any]]] = {
    'additiveParams': {
        'layerHeight': {
            'type': float,
            'min': 0.1,
            'max': 0.4,
            'unit': 'mm',
            'required': True,
        },
        'nozzleDiameter': {
            'type': float,
            'min': 0.2,
            'max': 1.0,
            'unit': 'mm',
            'required': True,
        },
        'nozzleTemperature': {
            'type': int,
            'min': 180,
            'max': 300,
            'unit': '°C',
            'required': True,
        },
        'infillDensity': {
            'type': int,
            'min': 10,
            'max': 100,
            'unit': '%',
            'required': True,
        },
        'printSpeed': {
            'type': int,
            'min': 10,
            'max': 200,
            'unit': 'mm/s',
            'required': True,
        },
        'shellThickness': {
            'type': float,
            'min': 0.8,
            'max': 10.0,
            'unit': 'mm',
            'required': False,
        },
        'printerModel': {
            'type': str,
            'required': True,
        }
    },

    'castingParams': {
        'alloyTemperature': {
            'type': float,
            'min': 100,
            'max': 400,
            'unit': '°C',
            'required': True,
        },
        'castingVolume': {
            'type': float,
            'min': 1,
            'max': 10000,
            'unit': 'cm³',
            'required': True,
        },
        'solidificationTime': {
            'type': int,
            'min': 60,
            'max': 3600,
            'unit': 's',
            'required': True,
        },
        'castingPressure': {
            'type': int,
            'min': 0,
            'max': 100,
            'unit': 'bar',
            'required': True,
        },
        'coolingMedium': {
            'type': str,
            'required': False,
        }
    },

    'subtractiveParams': {
        'machineModel': {
            'type': str,
            'required': True,
        },
        'spindleSpeed': {
            'type': int,
            'min': 100,
            'max': 50000,
            'unit': 'rpm',
            'required': True,
        },
        'feedRate': {
            'type': int,
            'min': 1,
            'max': 1000,
            'unit': 'mm/min',
            'required': True,
        },
        'depthOfCut': {
            'type': float,
            'min': 0.1,
            'max': 10.0,
            'unit': 'mm',
            'required': True,
        },
        'stepOver': {
            'type': float,
            'min': 0.1,
            'max': 50.0,
            'unit': '%',
            'required': True,
        }
    }
}

# Parameter dependency rules
dependencyRules: Dict[str, Any] = {
    # Logical consistency checks
    'additiveParams': {
        'shellThickness_vs_infillDensity': {
            'condition': 'shellThickness > 0',
            'mustHave': ['infillDensity'],
            'message': 'Must set infill density when shell thickness > 0'
        },
        'layerHeight_vs_nozzleDiameter': {
            'condition': 'layerHeight < nozzleDiameter',
            'message': 'Layer height should be less than nozzle diameter'
        }
    }
}


def main():
    """Test configuration validation rules."""
    print("=== Configuration Validation Rules Test ===")

    # Test enum classes
    print("1. Enum class test:")
    print(f"   InfillPattern.GRID = {InfillPattern.GRID.value}")
    print(f"   InfillPattern.HONEYCOMB = {InfillPattern.HONEYCOMB.value}")

    # Test validation rules
    print("\n2. Validation rule structure test:")
    print(f"   Modules included: {list(validationRules.keys())}")

    additiveRules = validationRules['additiveParams']
    print(f"   Additive manufacturing parameter count: {len(additiveRules)}")

    # Test dependency rules
    print("\n3. Dependency rule test:")
    for module, rules in dependencyRules.items():
        for ruleName, rule in rules.items():
            print(f"   {ruleName}: {rule.get('message', 'No message')}")

    print("\nTest completed!")


if __name__ == "__main__":
    main()