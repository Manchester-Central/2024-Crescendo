# 2024-Crescendo

## Troubleshooting Build Issues

### fixing junit unresolved dependency
```
Invoke-WebRequest -Uri https://repo.maven.apache.org/maven2/org/junit/jupiter/junit-jupiter/5.10.1/junit-jupiter-5.10.1.module -OutFile C:\Users\Public\wpilib\2024\maven\org\junit\jupiter\junit-jupiter\5.10.1\junit-jupiter-5.10.1.module
Invoke-WebRequest -Uri https://repo.maven.apache.org/maven2/org/junit/junit-bom/5.10.1/junit-bom-5.10.1.module -OutFile C:\Users\Public\wpilib\2024\maven\org\junit\junit-bom\5.10.1\junit-bom-5.10.1.module
```
run vscode `Clean Java Language Server Workspace` command

https://docs.wpilib.org/en/stable/docs/yearly-overview/known-issues.html

### Fixing `[...]/secrets.json (The system cannot find the file specified)` errors

Look at this section of the CHAOS Shared Code README for how to set up the secrets file needed for downloading the package:
https://github.com/Manchester-Central/CHAOS-Shared-Code?tab=readme-ov-file#secretsjson