# DDS QoS XML Generator

A tool for automatically generating XML files for DDS QoS test cases. It produces efficient test cases using **Pairwise testing**, covering combinations of 16 parameters with minimal redundancy.

---

## Table of Contents

- [Parameter List](#parameter-list)
- [Pairwise Test Case Generation](#pairwise-test-case-generation)
- [Final Case Count](#final-case-count)
- [How to Run](#how-to-run)
- [Reference](#reference)

---

## Download
[Download XML Generator Package (ZIP)](./downloads/XML_Generator_Tool.zip)

---

## Parameter List

### Overview

| Parameter | Pub Values | Pub Value Types | Sub Values | Sub Value Types | Constraint | Combinations |
|-----------|------------|-----------------|------------|-----------------|------------|--------------|
| **ENTITY_FACTORY** | 2 | True, False | 2 | True, False | pub=sub | **2** |
| **DATA_EXISTS** | 2 | exists, not_exists | 2 | exists, not_exists | pub=sub | **2** |
| **RELIABILITY** | 2 | RELIABLE, BEST_EFFORT | 2 | RELIABLE, BEST_EFFORT | Independent | **4** |
| **DURABILITY** | 4 | TRANSIENT_LOCAL, VOLATILE, TRANSIENT, PERSISTENT | 4 | TRANSIENT_LOCAL, VOLATILE, TRANSIENT, PERSISTENT | Independent | **16** |
| **DEADLINE** | 3 | PP, 2×PP, DURATION_INFINITY | 3 | PP, 2×PP, DURATION_INFINITY | Independent | **9** |
| **LIVELINESS** | 6 | (AUTOMATIC, 0.5×PP), (AUTOMATIC, PP), (AUTOMATIC, 2×PP), (AUTOMATIC, ∞), (MANUAL_BY_PARTICIPANT, None), (MANUAL_BY_TOPIC, None) | 6 | (same) | Independent | **36** |
| **HISTORY** | 5 | (KEEP_ALL, None), (KEEP_LAST, 1), (KEEP_LAST, &lt;(RTT/PP)+2), (KEEP_LAST, =(RTT/PP)+2), (KEEP_LAST, &gt;(RTT/PP)+2) | 5 | (same) | Independent | **25** |
| **RESOURCE_LIMITS_MAX_SAMPLES_PER_INSTANCE** | 4 | 1, &lt;(RTT/PP)+2, =(RTT/PP)+2, &gt;(RTT/PP)+2 | 4 | (same) | Independent | **16** |
| **RESOURCE_LIMITS_MAX_SAMPLES** | 2 | 1, (RTT/PP+3)×PP | 2 | (same) | Independent | **4** |
| **LIFESPAN** | 2 | 0.5×RTT, DURATION_INFINITY | 2 | (same) | Independent | **4** |
| **OWNERSHIP** | 2 | SHARED, EXCLUSIVE | 2 | (same) | Independent | **4** |
| **DESTINATION_ORDER** | 2 | BY_RECEPTION_TIMESTAMP, BY_SOURCE_TIMESTAMP | 2 | (same) | Independent | **4** |
| **WRITER_DATA_LIFECYCLE** | 2 | True, False | 0 | N/A (pub only) | pub only | **2** |
| **READER_DATA_LIFECYCLE_NO_WRITER** | 0 | N/A (sub only) | 2 | 0, 3 | sub only | **2** |
| **READER_DATA_LIFECYCLE_DISPOSED** | 0 | N/A (sub only) | 2 | 0, 3 | sub only | **2** |

### Parameter Details

#### 1. ENTITY_FACTORY
- **Constraint:** pub and sub must always have the same value.
- **Combinations:** (True, True), (False, False)

#### 2. DATA_EXISTS
- **Description:** Combined parameter for PARTITION, USER_DATA, GROUP_DATA, TOPIC_DATA.
- **Constraint:** pub and sub must always have the same value.
- **Combinations:** (exists, exists), (not_exists, not_exists)

#### 3. RELIABILITY
- **Independent:** pub and sub can be set independently.
- **Combinations:** 2 × 2 = 4

#### 4. DURABILITY
- **Independent:** pub and sub can be set independently.
- **Combinations:** 4 × 4 = 16

#### 5. DEADLINE
- **Values:**
  - `PP`: D &lt; 2×PP case
  - `2×PP`: D ≥ 2×PP case
  - `DURATION_INFINITY`: infinite
- **Independent:** pub and sub can be set independently.
- **Combinations:** 3 × 3 = 9

#### 6. LIVELINESS
- **Values:**
  - `(AUTOMATIC, 0.5×PP)`: lease_duration &lt; 2×PP
  - `(AUTOMATIC, PP)`: lease_duration &lt; 2×PP
  - `(AUTOMATIC, 2×PP)`: lease_duration ≥ 2×PP
  - `(AUTOMATIC, DURATION_INFINITY)`: infinite
  - `(MANUAL_BY_PARTICIPANT, None)`: manual type
  - `(MANUAL_BY_TOPIC, None)`: manual type
- **Independent:** pub and sub can be set independently.
- **Combinations:** 6 × 6 = 36

#### 7. HISTORY
- **Values:**
  - `(KEEP_ALL, None)`: KEEP_ALL
  - `(KEEP_LAST, 1)`: depth=1
  - `(KEEP_LAST, &lt;(RTT/PP)+2)`: depth &lt; (RTT/PP)+2
  - `(KEEP_LAST, =(RTT/PP)+2)`: depth = (RTT/PP)+2
  - `(KEEP_LAST, &gt;(RTT/PP)+2)`: depth &gt; (RTT/PP)+2
- **Independent:** pub and sub can be set independently.
- **Combinations:** 5 × 5 = 25

#### 8. RESOURCE_LIMITS_MAX_SAMPLES_PER_INSTANCE
- **Values:** `1`, `&lt;(RTT/PP)+2`, `=(RTT/PP)+2`, `&gt;(RTT/PP)+2`
- **Independent:** pub and sub can be set independently.
- **Combinations:** 4 × 4 = 16

#### 9. RESOURCE_LIMITS_MAX_SAMPLES
- **Values:** `1`, `(RTT/PP+3)×PP`
- **Independent:** pub and sub can be set independently.
- **Combinations:** 2 × 2 = 4

#### 10. LIFESPAN
- **Values:** `0.5×RTT`, `DURATION_INFINITY`
- **Independent:** pub and sub can be set independently.
- **Combinations:** 2 × 2 = 4

#### 11. OWNERSHIP
- **Values:** `SHARED`, `EXCLUSIVE`
- **Independent:** pub and sub can be set independently.
- **Combinations:** 2 × 2 = 4

#### 12. DESTINATION_ORDER
- **Values:** `BY_RECEPTION_TIMESTAMP`, `BY_SOURCE_TIMESTAMP`
- **Independent:** pub and sub can be set independently.
- **Combinations:** 2 × 2 = 4

#### 13. WRITER_DATA_LIFECYCLE
- **Description:** Applied to Publisher only.
- **Values:** `True`, `False`
- **Combinations:** 2

#### 14. READER_DATA_LIFECYCLE_NO_WRITER
- **Description:** Applied to Subscriber only.
- **Values:** `0`, `3`
- **Combinations:** 2

#### 15. READER_DATA_LIFECYCLE_DISPOSED
- **Description:** Applied to Subscriber only.
- **Values:** `0`, `3`
- **Constraint:** Must match READER_DATA_LIFECYCLE_NO_WRITER value.
- **Combinations:** 2

---

## Pairwise Test Case Generation

### What is Pairwise Testing?

**Pairwise testing** generates a minimal set of test cases that cover every pair of parameter values at least once. This reduces the total number of cases while maintaining good coverage.

### Generation Process

1. **Collect parameters**
   - Select only parameters that have pub or sub values among the 16 parameters.

2. **Generate combinations per parameter**
   - `_get_param_values()` builds pub/sub combinations for each parameter.
   - Combination count is determined by constraints.

3. **Apply Pairwise algorithm**
   - **With allpairspy (optional):** Uses an optimized algorithm.
   - **Without allpairspy:** Built-in simple Pairwise implementation:
     - Generate all parameter pairs: C(16,2) = 120 pairs
     - Build value combinations for each pair
     - Create test cases (default values + current pair values)
     - Remove duplicates

4. **Apply constraints**
   - READER_DATA_LIFECYCLE_NO_WRITER and READER_DATA_LIFECYCLE_DISPOSED must have the same value.
   - Exclude combinations that violate this.

### Example

**Parameters A, B, C with 2, 3, and 2 values respectively:**

- **Full combinations:** 2 × 3 × 2 = 12
- **Pairwise cases:** Minimal set covering all pairs (A–B, A–C, B–C) at least once (about 6–8 cases)

This yields effective test coverage with far fewer cases than exhaustive combination.

---

## Final Case Count

### Theoretical Full Combination Count

If all parameters were combined independently:

```
2 × 2 × 4 × 16 × 9 × 36 × 25 × 16 × 4 × 4 × 4 × 4 × 2 × 2 × 2
≈ 271 trillion
```

### Actual Generated Case Count

With the Pairwise algorithm:

- **Generated cases:** about **853** (for PP=0.1, RTT=0.2)
- **Efficiency:** about **0.0000003%** of full combination

### How the Count is Computed

1. **Per-parameter combination count**
   - Independent: `pub count × sub count`
   - Constrained: `pub count` (pub=sub) or `sub count` (pub-only/sub-only)

2. **Pairwise algorithm**
   - Build minimal set covering every parameter pair at least once.
   - Actual count depends on parameter interactions.

3. **Constraint filtering**
   - Remove cases that violate READER_DATA_LIFECYCLE constraints.

### Sample Run (PP=0.1, RTT=0.2)

```
Total: 853 test cases
XML files: 1,706 (853 cases × 2 files)
  - pub_qos_case_00001.xml ~ pub_qos_case_00853.xml
  - sub_qos_case_00001.xml ~ sub_qos_case_00853.xml
```

---

## How to Run

### Requirements

- **Python** 3.6 or higher
- **Required:**
  ```bash
  pip install xml
  ```
- **Optional** (optimized Pairwise):
  ```bash
  pip install allpairspy
  ```
  > The tool works without `allpairspy` using the built-in algorithm.

### Project Structure

```
XML Generator/
├── xml_generator.py      # Main script
├── xml/
│   ├── pub_qos.xml       # Publisher template
│   └── sub_qos.xml       # Subscriber template
└── output/               # Generated XML files
```

### Running the Generator

#### Option 1: Interactive

```bash
cd "XML Generator"
python3 xml_generator.py
```

You will be prompted for:

- **PP (Publication Period):** in seconds  
- **RTT (Round Trip Time):** in seconds  

#### Option 2: From code

```python
from xml_generator import XMLGenerator

generator = XMLGenerator(pp=0.1, rtt=0.2)
generator.generate_all_test_cases(output_dir='output')
```

#### Option 3: Non-interactive (script)

```bash
cd "XML Generator"
python3 xml_generator.py <<< $'0.1\n0.2\n'
```

### Sample Output

```
============================================================
DDS QoS XML Generator
============================================================
Enter PP (Publication Period) in seconds: 0.1
Enter RTT (Round Trip Time) in seconds: 0.2

Input values:
  PP: 0.1 s
  RTT: 0.2 s
  RTT/PP: 2.00
Generating test case combinations...
Total: 853 test cases

Generating XML files...
Progress: 100/853 (11%)
Progress: 200/853 (23%)
...
Progress: 800/853 (93%)

Done! 853 test cases generated.
Output directory: output/
```

### Output Files

Generated XML files are written to `output/`:

- **Publisher:** `pub_qos_case_00001.xml` ~ `pub_qos_case_00853.xml`
- **Subscriber:** `sub_qos_case_00001.xml` ~ `sub_qos_case_00853.xml`

Each file contains the DDS QoS profile with parameter values for that test case.

### Notes

- **PP:** Must be greater than 0.
- **Overwrite:** Existing files with the same name are overwritten.
- **Directory:** `output/` is created automatically if it does not exist.

---

## Reference

### Dynamic Value Calculation

Some parameter values depend on PP and RTT:

- **DEADLINE:** `PP`, `2×PP`
- **LIVELINESS:** `0.5×PP`, `PP`, `2×PP`
- **HISTORY:** based on `(RTT/PP)+2`
- **RESOURCE_LIMITS:** based on `(RTT/PP)+2`
- **LIFESPAN:** `0.5×RTT`

Case count and values may change with different PP and RTT.

### Constraints Summary

| Constraint | Description |
|------------|-------------|
| ENTITY_FACTORY, DATA_EXISTS | pub and sub must be equal |
| READER_DATA_LIFECYCLE | NO_WRITER and DISPOSED must be equal |
| WRITER_DATA_LIFECYCLE | Publisher only (sub is None) |
| READER_DATA_LIFECYCLE_* | Subscriber only (pub is None) |

---

## License

This project is a tool for generating DDS QoS test cases.
