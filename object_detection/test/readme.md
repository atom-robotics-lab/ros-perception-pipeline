1. Install `pytest-cov`

```bash
pip install pytest-cov
```

2. Run the tests
```bash
colcon test --event-handlers console_cohesion+ --pytest-args "--cov=object_detection --cov-report=xml --cov-report=html"
```

or use this command for more logs :

```bash
colcon test --pytest-args="-s"
```

3. Generate code coverage report

```bash
pytest --cov=object_detection --cov-report=xml --cov-report=html --cov-report=term
```
