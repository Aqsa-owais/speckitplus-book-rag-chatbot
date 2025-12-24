# Data Model: UI Error Diagnosis & Fix for Physical AI Humanoid Robotics Book

**Feature**: UI Error Diagnosis & Fix for Physical AI Humanoid Robotics Book
**Date**: 2025-12-25
**Branch**: 2-ui-error-fix

## Entities

### Page
- **Description**: Represents a book chapter or section that must render without errors
- **Properties**:
  - id: string (unique identifier for the page)
  - path: string (URL path to the page)
  - title: string (display title of the page)
  - content: string (the actual content of the page)
  - module: string (which module this page belongs to)

### ModuleTab
- **Description**: Represents a navigation element for switching between book modules
- **Properties**:
  - id: string (unique identifier for the tab, e.g. "module1", "home")
  - label: string (display text for the tab)
  - visible: boolean (whether the tab is currently visible)
  - active: boolean (whether the tab is currently selected)

### UIComponent
- **Description**: Represents a custom Docusaurus theme component that may be causing conflicts
- **Properties**:
  - name: string (component name)
  - type: string ("layout", "navigation", "content", "theme")
  - status: string ("working", "error", "fixed")
  - dependencies: string[] (other components or libraries it depends on)

### Style
- **Description**: Represents CSS rules that need to be properly applied without conflicts
- **Properties**:
  - id: string (unique identifier for the style rule)
  - selector: string (CSS selector)
  - properties: object (CSS property-value pairs)
  - source: string ("custom", "docusaurus-default", "plugin")
  - status: string ("applied", "conflicting", "missing")

## Relationships

- Page belongs to one Module
- ModuleTab controls visibility of multiple Pages
- UIComponent may use multiple Styles
- Page may contain multiple UIComponents

## State Transitions

### Page Error State
- **Initial State**: Page loads with errors ("This page crashed")
- **Action**: Apply UI fixes and error resolution
- **Transition**: JavaScript errors are resolved
- **Result**: Page loads successfully without crashes

### Module Tab Visibility
- **Initial State**: Module tabs are not visible in navbar
- **Action**: Fix tab component integration
- **Transition**: Module tabs become visible and functional
- **Result**: Users can navigate between modules using tabs

### UI Component Status
- **Initial State**: UIComponent status is "error"
- **Action**: Debug and fix component issues
- **Transition**: Component status changes to "fixed"
- **Result**: Component functions without errors