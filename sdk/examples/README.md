# Site API example

The example is a complete local simulated SDK deployment:

- site.yaml declares one simulated YAM arm, owner safety, recording, and paths.
- run_site.py opens hardware only inside the Site context, observes the arm,
  submits one bounded hold-position action, records success, and closes.

Run it from the sdk directory:

    uv run python examples/run_site.py

The same program can select a remote waddle.v0 control transport:

    transport = waddle_sdk.Grpc(
        "https://api.waddlelabs.ai:443",
        token,
        customer_id="customer",
        project_id="project",
        workspace_id="workspace",
    )
    with site.open(transport=transport) as session:
        ...

Metal owns task graphs, skills, and hosted-run orchestration. The SDK example
therefore demonstrates only the Site/SiteSession/Run hardware contract.
