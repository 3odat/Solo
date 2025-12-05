# Memory System Testing Guide - Phases 2-6

This guide provides step-by-step commands to manually test each phase of the memory system.

---

## Prerequisites

```bash
cd /home/px4/research/uav_project_AntiGravity

# Clean any existing test data
rm -rf data/
mkdir -p data
```

---

## Phase 2: Architecture & Schemas

### Test 2.1: View Episodic Schema
```bash
python3 -c "
import json
schema = {
    'id': 0,
    'timestamp': 1715000000.0,
    'iso_time': '2024-05-06T12:00:00',
    'agent_id': 'drone1',
    'task': 'scan_sector',
    'action': 'do_scan_sector',
    'outcome': 'success',
    'context': {
        'params': {'sector': 'A'},
        'soc': 95.5,
        'energy_used': 2.5,
        '_signature': 'hmac-sha256-here'
    }
}
print('Episodic Entry Schema:')
print(json.dumps(schema, indent=2))
"
```

### Test 2.2: View Semantic Schema
```bash
python3 -c "
import json
schema = {
    'rule_id': 0,
    'content': 'Avoid Sector B due to interference',
    'category': 'mission_constraint',
    'confidence': 0.95,
    'source': 'system',
    'active': True
}
print('Semantic Rule Schema:')
print(json.dumps(schema, indent=2))
"
```

---

## Phase 3: Baseline Multi-Agent System

### Test 3.1: Single Drone Baseline
```bash
# Run single drone mission
python3 -m src.single_drone_baseline
```
**Expected Output:**
- Drone1 performs: takeoff → scan_sector → land
- SOC decreases with each action
- Episodes logged to memory

### Test 3.2: Two-Drone Parallel Execution
```bash
# Clean data first
rm -rf data/ && mkdir data

# Run two-drone mission
python3 -m src.main --mission "Survey the area"
```
**Expected Output:**
- Stage 1: Both drones takeoff in parallel
- Stage 2: Both drones scan sectors A and B in parallel
- Stage 3: Both drones land in parallel
- 6 total tasks executed

### Test 3.3: Verify Structured Logging
```bash
# View episodic log
cat data/episodic.json | python3 -m json.tool
```
**Expected:** Each entry has `agent_id`, `task`, `action`, `outcome`, `context` with `soc`

### Test 3.4: Verify Semantic Memory
```bash
# View semantic rules
cat data/semantic.json | python3 -m json.tool
```
**Expected:** At least one rule (e.g., "Fly above 5m")

---

## Phase 4: Memory Layer Implementation

### Test 4.1: Write Episodic Entry
```bash
python3 -c "
from src.memory.memory_interface import MemoryInterface
import os
os.makedirs('data', exist_ok=True)
memory = MemoryInterface('data/test_ep.json', 'data/test_sem.json')
ep_id = memory.log_episode('test_drone', 'test_task', 'do_test', 'success', {'soc': 88.0})
print(f'✓ Created episode with ID: {ep_id}')
"
```

### Test 4.2: Query Episodic Memory
```bash
python3 -c "
from src.memory.memory_interface import MemoryInterface
memory = MemoryInterface('data/test_ep.json', 'data/test_sem.json')
episodes = memory.recall_episodes({'agent_id': 'test_drone'}, limit=5)
print(f'✓ Found {len(episodes)} episodes')
for ep in episodes:
    print(f'  - [{ep[\"agent_id\"]}] {ep[\"task\"]} -> {ep[\"outcome\"]}')
"
```

### Test 4.3: Write Semantic Rule
```bash
python3 -c "
from src.memory.memory_interface import MemoryInterface
memory = MemoryInterface('data/test_ep.json', 'data/test_sem.json')
rule_id = memory.add_rule('Test rule', 'test_category', source='manual')
print(f'✓ Created rule with ID: {rule_id}')
"
```

### Test 4.4: Query Semantic Rules
```bash
python3 -c "
from src.memory.memory_interface import MemoryInterface
memory = MemoryInterface('data/test_ep.json', 'data/test_sem.json')
rules = memory.get_rules('test_category')
print(f'✓ Found {len(rules)} rules')
for r in rules:
    print(f'  - [{r[\"id\"]}] {r[\"content\"]}')
"
```

---

## Phase 5: Attack Harness

### Test 5.1: Inject Fake Obstacle Attack
```bash
# Clean data
rm -rf data/ && mkdir data

# Inject attack
python3 -c "
from src.memory.memory_interface import MemoryInterface
from src.attacks.attack_harness import AttackHarness
import os
os.makedirs('data', exist_ok=True)
memory = MemoryInterface('data/episodic.json', 'data/semantic.json')
attacker = AttackHarness(memory)
attacker.inject_episodic_poison('fake_obstacle', 'scan_sector', count=1, context_override={'params': {'sector': 'A'}})
print('✓ Injected fake obstacle for Sector A')
"
```

### Test 5.2: Inject Bad Rule Attack
```bash
python3 -c "
from src.memory.memory_interface import MemoryInterface
from src.attacks.attack_harness import AttackHarness
memory = MemoryInterface('data/episodic.json', 'data/semantic.json')
attacker = AttackHarness(memory)
attacker.inject_semantic_poison('Avoid Sector B', 'mission_constraints')
print('✓ Injected bad rule: Avoid Sector B')
"
```

### Test 5.3: Verify Attack Impact
```bash
# Run mission - should skip sectors due to attacks
python3 -m src.main --mission "Survey the area"
```
**Expected Output:**
- `[supervisor] Found past failure in Sector A`
- `[supervisor] modifying plan to avoid sectors: ['B', 'A']`
- `Executing Stage 2 with 0 tasks...` (NO scanning!)

---

## Phase 6: Defense Layer

### Test 6.1: Test Integrity Manager (HMAC)
```bash
python3 -c "
from src.defense.integrity import IntegrityManager
integrity = IntegrityManager()

# Sign data
data = {'agent_id': 'drone1', 'task': 'test', 'outcome': 'success'}
signature = integrity.sign_data(data)
print(f'Signature: {signature[:40]}...')

# Verify original
is_valid = integrity.verify_data(data, signature)
print(f'Original verification: {\"PASSED\" if is_valid else \"FAILED\"}'

# Verify tampered
data['outcome'] = 'failed'
is_valid = integrity.verify_data(data, signature)
print(f'Tampered verification: {\"BLOCKED\" if not is_valid else \"PASSED\"}'
"
```

### Test 6.2: Test Semantic Validator
```bash
python3 -c "
from src.defense.consistency import SemanticValidator
validator = SemanticValidator()

# Valid rule
result = validator.validate_rule('Fly above 5m', 'mission_constraints')
print(f'Valid rule: {\"ACCEPTED\" if result else \"REJECTED\"}')

# Invalid rule (blocks safe sector)
result = validator.validate_rule('Avoid Sector A', 'mission_constraints')
print(f'Invalid rule: {\"ACCEPTED\" if result else \"REJECTED\"}')
"
```

### Test 6.3: Full Defense Test (Attack + Defense)
```bash
# Clean data
rm -rf data/ && mkdir data

# Inject attack AND run with defense
python3 -m src.main --attack fake_obstacle --defense
```
**Expected Output:**
- `[Attack] Injecting 1 episodes...`
- `[Defense] Blocked unsigned episode...`
- Mission proceeds normally with 2 scan tasks

### Test 6.4: Compare Attack Only vs Attack + Defense
```bash
# Clean and run WITHOUT defense
rm -rf data/ && mkdir data
python3 -m src.main --attack fake_obstacle
echo "---"
echo "Scan tasks WITHOUT defense: Check output above"

# Clean and run WITH defense
rm -rf data/ && mkdir data
python3 -m src.main --attack fake_obstacle --defense
echo "---"
echo "Scan tasks WITH defense: Check output above"
```

---

## Quick Reference Commands

| Test | Command |
|------|---------|
| Clean data | `rm -rf data/ && mkdir data` |
| Single drone | `python3 -m src.single_drone_baseline` |
| Two drones | `python3 -m src.main` |
| Custom mission | `python3 -m src.main --mission "Your mission"` |
| Attack only | `python3 -m src.main --attack fake_obstacle` |
| Defense only | `python3 -m src.main --defense` |
| Attack + Defense | `python3 -m src.main --attack fake_obstacle --defense` |
| Bad rule attack | `python3 -m src.main --attack bad_rule` |
| Full test suite | `python3 tests/test_memory_phases.py` |

---

## Troubleshooting

### ModuleNotFoundError
```bash
# Always run from project root with -m flag
cd /home/px4/research/uav_project_AntiGravity
python3 -m src.main
```

### Stale memory affecting results
```bash
# Always clean before fresh tests
rm -rf data/ && mkdir data
```
