# Research: Docusaurus UI Update for Physical AI Robotics Book

**Feature**: Docusaurus UI Update for Physical AI Robotics Book
**Date**: 2025-12-25
**Branch**: 1-docusaurus-ui-update

## Decision: Docusaurus Tab-Based Navigation Implementation

### Rationale:
To implement tab-based navigation in Docusaurus for the 4 modules, I'll need to create custom React components that can:
1. Show/hide content based on selected tab
2. Update the sidebar dynamically based on selected module
3. Maintain proper routing and URL structure
4. Work with Docusaurus's existing theme and layout system

### Implementation Approach:
- Create a custom layout wrapper that manages the tab state
- Use React hooks to track which module is selected
- Conditionally render content based on selected tab
- Update sidebar through Docusaurus context or plugin

### Alternatives Considered:
1. **Separate pages per module**: Create separate routes for each module (e.g., /module1, /module2) - rejected because it doesn't provide the tabbed interface experience requested
2. **Docusaurus docs plugin with categories**: Use built-in category features - rejected because it doesn't provide the specific tabbed navigation behavior requested
3. **Custom theme components**: Override Docusaurus theme with custom navbar and layout components - selected as the approach

## Decision: Styling Implementation

### Rationale:
For implementing the specified design system (colors, typography, shadows, etc.), I'll use:
- Docusaurus's CSS variable system for color palette
- Custom CSS for typography overrides
- SCSS/CSS modules for component-specific styling
- Docusaurus theme configuration for light/dark mode

### Implementation Approach:
- Define CSS variables in the theme for the color palette
- Override Docusaurus default styles with custom CSS
- Use Docusaurus's theme configuration for typography
- Implement responsive design with CSS media queries

### Alternatives Considered:
1. **CSS-in-JS**: Using styled-components or emotion - rejected as it adds complexity for a documentation site
2. **Tailwind CSS**: Adding Tailwind framework - rejected as it may conflict with Docusaurus's existing styling system
3. **Pure CSS overrides**: Using Docusaurus's custom CSS approach - selected as the approach

## Decision: Sidebar Dynamic Behavior

### Rationale:
To achieve the dynamic sidebar that updates based on selected module tab, I'll need to:
- Create a custom sidebar component that can filter items based on current module
- Use Docusaurus's sidebar data structure to dynamically show/hide sections
- Maintain proper active state highlighting for current chapter

### Implementation Approach:
- Create a custom sidebar component that accepts module parameter
- Use Docusaurus's useDocsSidebar hook to access sidebar data
- Filter sidebar items based on selected module
- Update active state based on current page and selected module

### Alternatives Considered:
1. **Multiple sidebar files**: Create separate sidebar configurations - rejected as it doesn't provide the dynamic switching behavior
2. **Plugin approach**: Create a Docusaurus plugin for this functionality - rejected as it's overkill for this specific feature
3. **Custom component**: Build custom sidebar component - selected as the approach

## Decision: Breadcrumb Navigation

### Rationale:
For implementing breadcrumb navigation showing "Home → Module X → Chapter", I'll need to:
- Create a custom breadcrumb component
- Determine current context (home vs module view)
- Display appropriate path based on navigation state

### Implementation Approach:
- Create React component that determines path based on current route and selected module
- Use Docusaurus's route context to understand current location
- Display breadcrumbs according to specification

### Alternatives Considered:
1. **Docusaurus built-in breadcrumbs**: Use default Docusaurus breadcrumbs - rejected as they don't support the custom tab-based hierarchy
2. **Custom breadcrumb component**: Build from scratch - selected as the approach

## Decision: Footer Implementation

### Rationale:
To implement the strict footer requirement with only specific text, I'll:
- Override the default Docusaurus footer component
- Create a simple, minimal footer with the specified content
- Apply requested styling (center alignment, muted color, small font)

### Implementation Approach:
- Create custom footer component
- Replace default Docusaurus footer through theme configuration
- Apply minimal styling as specified

### Alternatives Considered:
1. **CSS override**: Hide default footer and add custom content - rejected as it's not clean
2. **Theme component override**: Replace footer component - selected as the approach