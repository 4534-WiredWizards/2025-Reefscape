# Reefscape 2025 Robot Code Repository

## 🤖 Repository Structure

### Branch Strategy
Base branch structure:
- `Gillbert`: Swerve base code
- `Subsystems`: All robot subsystem development
- `Commands`: All robot command development
- `Main`: Final integration branch

### Branch Management
- [ ] Set up branch protection rules
- [ ] All team members can work on `Subsystems` and `Commands` branches
- [ ] Max to review all Pull Requests before merging to main
- [ ] Branches will remain active for continuous development
- [ ] Limit base branch modifications to Max and Greg

## 🛠️ Code Setup

### Drivetrain
- [ ] Import Advantage Kit drive code
- [ ] Complete full diagnostic tuning for swerve

### Path Planning
- [ ] Evaluate and select Path Planner or Choreo
- [ ] Integrate chosen path planning solution

### Vision Integration
- [ ] Implement Limelight setup from Advantage Kit base code
- [ ] Configure forward and rear-facing camera
- [ ] Decision: Use Limelight (preferred from previous year)

## 📋 Additional Configuration

### Command Structure
- [ ] Develop base structure and constants
- [ ] Reference command list: [Reefscape Command Spreadsheet](https://docs.google.com/spreadsheets/d/1-hmzdXKS6qgACqeQ6U8RYg2TpfawPsJiIGIwLDxqnBw/edit?gid=1163826648#gid=1163826648)

## 🚀 Next Steps
- Coordinate with team on specific implementation details
- Regularly review and update this README

## Resources
- [Advantage Kit Vision Integration Docs](https://docs.advantagekit.org/getting-started/template-projects/talonfx-swerve-template/#vision-integration)