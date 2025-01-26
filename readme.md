# Reefscape 2025 Robot Code Repository

## 🤖 Repository Structure

### Branch Strategy
Base branch structure:
- `Gillbert`: Swerve base code
- `develop`: Active development and integration
- `main`: Stable, compitition-ready code 🤖🥳

### Branch Management
1. Set up branch protection rules
2. Team development occurs in `develop` branch
3. Max reviews all Pull Requests
4. Merge from `develop` to `main` only after thorough testing
5. Limit base branch modifications to Max and Greg

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
- Coordinate team development in `develop` branch
- Ensure comprehensive testing before merging to `main`
- Regularly review and update this README

## Resources
- [Advantage Kit Vision Integration Docs](https://docs.advantagekit.org/getting-started/template-projects/talonfx-swerve-template/#vision-integration)