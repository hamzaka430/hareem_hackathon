# Project Constitution

> Guiding principles for code quality, testing, UX, and performance

## 1. Code Quality Principles

### 1.1 Readability First
- **Write for humans, not machines.** Code is read far more often than it's written.
- Use clear, descriptive names for variables, functions, and classes.
- Prefer explicit over clever. Avoid obscure shortcuts or overly terse syntax.
- Keep functions small and focused on a single responsibility.

### 1.2 Consistency
- Follow established project conventions and style guides.
- Use automated formatters and linters to enforce consistency.
- When in doubt, match the existing codebase style.

### 1.3 Maintainability
- **DRY (Don't Repeat Yourself)**: Extract common logic into reusable functions/modules.
- **SOLID principles**: Write code that is easy to extend and modify.
- Document complex logic with clear comments explaining the "why", not just the "what".
- Keep dependencies minimal and well-justified.

### 1.4 Error Handling
- Handle errors gracefully with informative messages.
- Fail fast and explicitly. Avoid silent failures.
- Use appropriate error types and hierarchies.
- Log errors with sufficient context for debugging.

### 1.5 Code Review Standards
- All code must be reviewed by at least one other developer.
- Reviews should check for correctness, readability, test coverage, and architectural fit.
- Provide constructive, actionable feedback.
- Address all review comments before merging.

## 2. Testing Standards

### 2.1 Test Coverage Requirements
- **Minimum coverage**: 80% for new code, 70% for legacy code.
- 100% coverage for critical business logic and security-sensitive code.
- Coverage metrics should include branch coverage, not just line coverage.

### 2.2 Testing Pyramid
- **Unit Tests (70%)**: Fast, isolated tests for individual functions/methods.
- **Integration Tests (20%)**: Test interactions between components/modules.
- **End-to-End Tests (10%)**: Test complete user workflows.

### 2.3 Test Quality
- Tests must be deterministic and reliable (no flaky tests).
- Each test should have a clear purpose and test one thing.
- Use descriptive test names that explain what is being tested and expected outcome.
- Follow the **AAA pattern**: Arrange, Act, Assert.
- Keep tests independent—no shared state between tests.

### 2.4 Test-Driven Development (TDD)
- Write tests before implementation for new features when appropriate.
- Use tests to drive design and clarify requirements.
- Refactor with confidence knowing tests will catch regressions.

### 2.5 Continuous Testing
- All tests must pass before merging to main branch.
- Run tests automatically on every commit (CI/CD pipeline).
- Performance tests should run on a regular schedule.

## 3. UX Consistency

### 3.1 Design System
- Use a unified design system with consistent components, colors, typography, and spacing.
- Document all UI components in a component library/style guide.
- No custom one-off components without design approval.

### 3.2 User Interface Standards
- **Accessibility First**: Follow WCAG 2.1 AA standards minimum.
  - Proper semantic HTML
  - Keyboard navigation support
  - Screen reader compatibility
  - Sufficient color contrast (4.5:1 for normal text)
- **Responsive Design**: Support mobile, tablet, and desktop viewports.
- **Touch-friendly**: Minimum 44x44px touch targets for interactive elements.

### 3.3 Interaction Patterns
- Consistent navigation across all pages/views.
- Standard form validation and error messaging patterns.
- Predictable button placement and behavior.
- Use familiar UI patterns—don't reinvent common interactions.

### 3.4 Feedback & Communication
- Provide immediate feedback for user actions (loading states, confirmations).
- Error messages must be clear, actionable, and user-friendly.
- Success states should be visually distinct and affirming.
- Use progressive disclosure to avoid overwhelming users.

### 3.5 Performance Perception
- Show loading indicators for operations taking >200ms.
- Use optimistic UI updates where appropriate.
- Skeleton screens for content-heavy pages.
- Prioritize perceived performance (what users see) as much as actual performance.

## 4. Performance Requirements

### 4.1 Page Load Performance
- **First Contentful Paint (FCP)**: < 1.5 seconds
- **Largest Contentful Paint (LCP)**: < 2.5 seconds
- **Time to Interactive (TTI)**: < 3.5 seconds
- **Cumulative Layout Shift (CLS)**: < 0.1
- **First Input Delay (FID)**: < 100ms

### 4.2 Runtime Performance
- 60 FPS for animations and interactions (frame budget: 16.67ms).
- API response times: < 200ms for critical endpoints, < 1s for non-critical.
- Database queries: < 100ms for simple queries, < 500ms for complex queries.
- No blocking operations on the main thread.

### 4.3 Resource Optimization
- **JavaScript bundles**: < 200KB initial load (gzipped).
- **Images**: Properly sized, compressed, and lazy-loaded.
- **CSS**: Critical CSS inlined, non-critical deferred.
- **Fonts**: Subset fonts, use font-display: swap.
- **Caching**: Leverage browser and CDN caching appropriately.

### 4.4 Scalability
- Design for horizontal scalability from day one.
- Use asynchronous processing for heavy operations.
- Implement rate limiting and throttling for APIs.
- Monitor and optimize database queries and indexes.

### 4.5 Monitoring & Benchmarking
- Track Core Web Vitals in production using Real User Monitoring (RUM).
- Set up automated performance budgets in CI/CD.
- Run performance tests regularly against production-like environments.
- Alert on performance regressions before they reach users.

## 5. Enforcement

### 5.1 Automated Checks
- Linters and formatters run on pre-commit hooks.
- CI/CD pipeline enforces test coverage and passing tests.
- Performance budgets enforced in build process.
- Accessibility audits run on every build.

### 5.2 Regular Reviews
- Quarterly architecture and code quality reviews.
- Monthly performance audits.
- Regular accessibility audits with diverse user testing.

### 5.3 Continuous Improvement
- Retrospectives after each sprint/milestone to identify improvements.
- Keep this constitution as a living document—update as we learn.
- Share learnings and best practices across the team.

---

**Version**: 1.0  
**Last Updated**: December 24, 2025  
**Status**: Active
